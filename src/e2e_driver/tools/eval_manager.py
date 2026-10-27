#!/usr/bin/env python3
"""Closed-loop evaluation manager: the operators' side of a competition run, kept apart from the stack.

The stack (run.sh, or two_pc.sh up) only drives. It is told nothing about laps or returns and notices them itself
(e2e_driver route.TeleportDetector). This tool runs on the Mac next to two_pc.sh and controls MORAI on bc-250: MSC over
ssh (scene loads, play, pause), MORAI's official programs on bc-250 (checkpoint returns, weather and time) and gRPC on
192.168.0.15:7789 (Ego state).

    tools/eval.sh NAME [--laps N] [--seed S] [--weather SUNNY:13]      after two_pc.sh up and two_pc.sh prep

Laps: lap k gets a fresh random variant of the clcol scene, seed S*1000+k (scenario_variants.py on bc-250: Ego at
START; signal phases, NPC traffic, static obstacles, pedestrian and the GPS shadow area drawn at random). It is loaded
paused, the weather is set and play starts. A lap ends at the finish (the 14 checkpoints in order, then the last route
waypoint), at the 15 min limit, or when the route's 5 % point is not passed within 1 min of the start (rules §3-2);
the next lap's scene is loaded right away. Checkpoints: the official 10/7 table (checkpoints_20261007.csv, 3 m radius).
Weather and time (rules §2-7-2): per checkpoint section (number of checkpoints passed), (Sunny | Foggy density 1) x
(11 | 13 | 15 h) drawn from (lap seed, section) with env_follow.combo, set with MORAI's Time / Weather Changer
(WeatherTimeChanger_Linux, 9/28; its buttons pressed by wtc.py) and read back over gRPC. The program sends Weather.type
and SimulationTime.hour only: Foggy density and distance are the simulator's own Foggy values. Time Scale 1, Target
FPS 60 and Rendering High stay as the simulator presets set them.
Returns (rules §3-2: to the previous checkpoint, +15 s, when the operators judge the drive inoperable: all four wheels
off the drivable area outside the shadow zone, stuck after a collision, or the checkpoint order broken). Here:
  - off road: all four wheels off the lanelets and the 2 m route corridor (tools/scorer's drive_inoperable test)
    outside the shadow zone for OFF_ROAD_S;
  - stuck: less than PROGRESS_M of route progress for CRASH_STUCK_S after a collision, or for STUCK_S at all;
  - order broken: route progress SKIP_M past the next checkpoint without having come within 3 m of it.
The Ego goes to the last checkpoint passed (START before CP1) with MORAI's checkpoint Spawn program (MORAI_Spawn_Linux,
10/13: point number on stdin; SetTransform to the 10/7 table pose, z and yaw as listed, speed left as it is).
Official programs: tools/fetch_official_tools.sh (morai-docs originals, SHA-256 checked). --direct, or a failed
official call, uses the same gRPC requests from here instead (recorded per lap as "grpc").
Records (OUT/NAME/): laps.jsonl, one line per lap; lapK.rows.json.gz, the 20 Hz Ego states the lap was scored from by
Alpamayo-Cosmos tools/scorer (collision, signal, speed and lane penalties; checkpoint, start and finish judgments).
Clock: bc-250 runs below real time and gRPC gives only the host clock (GetTimestamp = host microseconds), so rows and
lap times are on the simulator clock estimated from the Ego state: rho = sum |d pos| / sum |v| dt over the trailing
5 s while moving (the stack's sim_clock idea with exact positions). Wall-clock lap times are recorded too.
Viewer: http://localhost:8091 shows the simulator window (viewer.py on bc-250: MJPEG on 192.168.0.15:8090) and the
lap status.
"""
import argparse
import base64
import filecmp
import gzip
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import grpc
import numpy as np
from google.protobuf import descriptor_pool, json_format, message_factory

from env_follow import combo, get as get_env, send as send_weather

HERE = Path(__file__).resolve().parent
MAP = HERE.parent.parent / "map"
BC, HOST = "bc-250", "192.168.0.15"      # BC "local": commands run here (the simulator host itself)
LAUNCHER = "~/Workspaces/scratches/morai/MoraiLauncher_Lin_Data"
SCEN = LAUNCHER + "/SaveFile/Scenario/R_KR_PR_K-city_2025"
ACTOR = "morai_sim_api.actor.Actor"
EGO = {"id": {"value": "Ego"}, "object_type": "OBJECT_TYPE_VEHICLE", "client_key": ""}   # any key works on a launcher sim

CP_R, LIMIT_S, START_S, START_FRAC, RETURN_S = 3.0, 900.0, 60.0, 0.05, 15.0    # rules §3-2
FINISH_M = 1.0          # route progress within this of the last waypoint = finished (tools/scorer time.finish_m)
STUCK_S = 60.0          # shortcut: longer than any give-way wait seen (red lights pause it); signal cycles are unpublished
CRASH_STUCK_S = 15.0    # "cannot go on after a collision"
PROGRESS_M = 5.0        # route progress that resets the stuck timers: a car pushing a small box creeps ~0.1 m/s (10/26)
OFF_ROAD_S = 2.0
SKIP_M = 10.0
GRACE_S = 10.0          # no failure checks right after a start or a return: the stack starts over from standstill
HZ = 20.0
VIEW_FPS, VIEW_WIDTH = os.environ.get("VIEW_FPS", "5"), os.environ.get("VIEW_WIDTH", "960")   # 15 1280 for a lap video
VIEW_URL = f"http://{HOST}:8090/"
# EVAL_SIM=w1: the W1 copy on winpc (two_pc.sh SIM=w1, gRPC on the cable address). Scenes are written from WSL into its
# scenario folder, MSC goes through windows_msc.ps1, returns and weather use gRPC (the official programs here are the
# Linux builds), and winview.ps1 serves the window on winpc's loopback, forwarded to the Mac's 127.0.0.1:8090.
SIM = os.environ.get("EVAL_SIM", "bc250")
if SIM == "w1":
    BC, HOST, VIEW_URL = "winpc-wsl", "192.168.0.1", "http://127.0.0.1:8090/"
    W1_DATA = "C:/morai/multi-03-20261007/MoraiLauncher_Win_Data"
    SCEN = "/mnt/c/morai/multi-03-20261007/MoraiLauncher_Win_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
GRPC = HOST + ":7789"                     # --host / --grpc / --launcher: one world of several on a host (tools/pod/eval3.sh)
OFFICIAL = "~/morai-eval-official"      # on bc-250 (fetch_official_tools.sh)
SPAWN_SHA = "5ecc827f5fcbd1cbe00aabb5b01aa01e6d1f87293345d61c20ba5d139210a2f6"    # morai-docs assets/catalog.json
WTC_SHA = "8c47f48a8d54b79b657920a661f582bd587c9f9065b6b55c9e911d42a0369609"


class Sim:
    """MORAI gRPC through the simulator build's reflected descriptors (morai_native_protos.json)."""

    def __init__(self, host):
        self.pool = descriptor_pool.DescriptorPool()
        pending = [base64.b64decode(s) for s in json.load(open(HERE / "morai_native_protos.json"))]
        while pending:                                    # files import each other: add until all resolve
            rest = []
            for d in pending:
                try:
                    self.pool.AddSerializedFile(d)
                except Exception:
                    rest.append(d)
            assert len(rest) < len(pending), "unresolvable proto descriptors"
            pending = rest
        self.ch = grpc.insecure_channel(host)

    def call(self, method, req, svc=ACTOR, timeout=5.0):
        d = self.pool.FindServiceByName(svc).FindMethodByName(method)
        rq, rs = message_factory.GetMessageClass(d.input_type), message_factory.GetMessageClass(d.output_type)
        rpc = self.ch.unary_unary(f"/{svc}/{method}", request_serializer=lambda m: m.SerializeToString(),
                                  response_deserializer=rs.FromString)
        return json_format.MessageToDict(rpc(json_format.ParseDict(req, rq()), timeout=timeout),
                                         preserving_proto_field_name=True, use_integers_for_enums=True)

    def ego(self):
        """transform = base_link (rear axle, 1.5 m behind the bounding box centre), map frame; rotation z = heading deg.
        global_velocity is in km/h (25.3 while competition_status said 7.03 m/s, 10/26)."""
        m = self.call("GetActorState", EGO)
        tf = m.get("transform", {})
        loc, rot, vs = tf.get("location", {}), tf.get("rotation", {}), m.get("vehicle_state", {})
        yaw, gv = math.radians(rot.get("z", 0.0)), m.get("global_velocity", {})
        return dict(x=loc.get("x", 0.0), y=loc.get("y", 0.0), yaw=yaw,
                    v=(gv.get("x", 0.0) * math.cos(yaw) + gv.get("y", 0.0) * math.sin(yaw)) / 3.6,   # km/h
                    hit=list(vs.get("collision_objects", [])), tl=vs.get("tl_id", {}).get("value"),
                    color=vs.get("tl_color", -1))      # TrafficLightColor = MORAI trafficLightStatus bits

    def put(self, x, y, z, yaw_deg):
        stop = {"actor_info": EGO, "velocity": 0.0}
        self.call("SetVelocity", stop)
        r = self.call("SetTransform", {"actor_info": EGO, "transform": {"location": {"x": x, "y": y, "z": z},
                                                                          "rotation": {"x": 0.0, "y": 0.0, "z": yaw_deg}}})
        self.call("SetVelocity", stop)
        return r


class Clock:
    """simulator time estimated from the Ego state (see the module doc); held while standing or across jumps."""

    def __init__(self, rho=0.8):
        self.tau, self.rho, self.win, self.last = 0.0, rho, [], None

    def step(self, t, x, y, v):
        if self.last is not None:
            t0, x0, y0, v0 = self.last
            d = math.hypot(x - x0, y - y0)
            # moving steps only: a stop or start step's trapezoid v dt would bias the last estimate held while standing
            if d < 5.0 and min(abs(v), abs(v0)) > 0.5:     # d >= 5 m: a return or a scene load
                self.win.append((t, d, 0.5 * (abs(v) + abs(v0)) * (t - t0)))
            self.win = [w for w in self.win if w[0] > t - 5.0]
            den = sum(w[2] for w in self.win)
            if den > 2.0:
                self.rho = min(max(sum(w[1] for w in self.win) / den, 0.2), 1.05)
            self.tau += self.rho * (t - t0)
        self.last = (t, x, y, v)
        return self.tau


class Track:
    """progress s along the global path (~0.5 m vertices): window search from the last vertex, global after a jump."""

    def __init__(self, path):
        self.P, self.i = path, 0
        self.s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]

    def update(self, x, y):
        lo = max(self.i - 60, 0)
        w = np.hypot(*(self.P[lo:self.i + 400] - (x, y)).T)
        if w.min() > 15.0:
            lo, w = 0, np.hypot(*(self.P - (x, y)).T)
        self.i = lo + int(w.argmin())
        return float(self.s[self.i]), float(w.min())


def shade_test(area):
    """inside(x, y) for the scene's GPS shadow box. Its long side (size x) runs along rot yaw + 90 deg: so placed, the
    clcol box covers route s 1880.7-1977.7 (the tunnel); along yaw it would cross the road."""
    cx, cy = area["pos"]["x"], area["pos"]["y"]
    a = math.radians(float(area["rot"]["yaw"]) + 90.0)
    hl, hw = float(area["size"]["x"]) / 2, float(area["size"]["y"]) / 2
    c, s = math.cos(a), math.sin(a)
    return lambda x, y: abs((x - cx) * c + (y - cy) * s) <= hl and abs(-(x - cx) * s + (y - cy) * c) <= hw


def ssh(cmd, timeout=150):
    return subprocess.run(["bash", "-c", cmd] if BC == "local" else ["ssh", BC, cmd], capture_output=True, text=True,
                          timeout=timeout).stdout


def msc(action, option="", timeout=30):
    if SIM == "w1":                                   # windows_msc.ps1 prints one JSON line; msc.py's form is "ok {...}"
        out = subprocess.run(["ssh", "-o", "BatchMode=yes", "winpc-win", "powershell -NoProfile -ExecutionPolicy Bypass -File"
                              f" C:/morai-par/windows_msc.ps1 -Action {action} {'-Option ' + option if option else ''}"
                              f" -Timeout {min(timeout, 120)} -LinkFile {W1_DATA}/SaveFile/LinkConnection.txt"],
                             capture_output=True, text=True, timeout=timeout + 30).stdout.replace("\0", "").strip()
        line = out.splitlines()[-1] if out else "{}"
        try:
            return ("ok " if json.loads(line).get("ok") else "failed ") + line
        except ValueError:
            return "failed " + line
    out = ssh(f"python3 ~/morai-eval-msc.py {LAUNCHER}/SaveFile/LinkConnection.txt {action} '{option}' {timeout}",
              timeout + 30).strip().splitlines()
    return out[-1] if out else ""


def make_scene(seed):
    """-> (scene name, shadow test, scenario_variants manifest entry): lap variant generated on bc-250."""
    out = ssh(f"cd {SCEN} && python3 ~/morai-eval-scenario-variants.py . clcol 1 {seed} ~/morai-eval-global-path.txt ev"
              f" > /dev/null && cat ev{seed}_000.json && echo && echo @@ && cat ev{seed}-manifest.json")
    scene, manifest = out.split("\n@@\n")
    name = f"ev{seed}_000"
    return name, shade_test(json.loads(scene)["shadedAreaList"][0]), json.loads(manifest)[name]


class Manager:
    def __init__(self, a):
        self.a, self.sim, self.clock = a, Sim(GRPC), Clock()
        sys.path.insert(0, a.scorer)
        from bag import episode_from_rows
        from scorer import KMap, load_config, score
        self.cfg = load_config(Path(a.scorer) / "thresholds.yaml")
        self.cfg["map"]["checkpoints"] = str(HERE / "checkpoints_20261007.csv")   # absolute: wins over map.dir
        self.kmap, self.episode_from_rows, self.score = KMap.from_files(self.cfg), episode_from_rows, score
        self.cps = np.loadtxt(HERE / "checkpoints_20261007.csv", delimiter=",", skiprows=1)   # START, CP1-14: x y z yaw
        self.track = Track(self.kmap.path)
        self.cp_i = [int(np.argmin(np.hypot(*(self.kmap.path - c[:2]).T))) for c in self.cps]   # START -> vertex 0
        veh = self.cfg["vehicle"]
        self.wheels = np.array(list(veh["wheels_base_link_m"].values()))
        self.out = Path(a.out).expanduser() / a.name
        self.out.mkdir(parents=True, exist_ok=True)
        self.status, self.history = "starting", []

    def off_road(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        w = np.c_[x + c * self.wheels[:, 0] - s * self.wheels[:, 1], y + s * self.wheels[:, 0] + c * self.wheels[:, 1]]
        return not (self.kmap.on_road(w).any() or self.kmap.near_path(w, self.cfg["inoperable"]["route_corridor_m"]).any())

    def weather(self, lap, seed, section, t):
        w, h = self.a.weather.split(":") if self.a.weather else combo(seed, section, 0)
        if self.a.weather and lap["weather"]:
            return
        how = "official"
        if self.a.direct or not self.official_weather(w, int(h)):
            send_weather(GRPC, w, int(h))
            how = "grpc"
        lap["weather"].append([section, w, int(h), round(t, 2), how])

    def official_weather(self, w, h):
        ssh(f"python3 ~/morai-eval-wtc.py set {w} {h}", 30)
        for _ in range(20):                               # the program sets them from a worker thread
            if get_env(GRPC) == (w, h):
                return True
            time.sleep(0.1)
        print(f"official weather/time program did not set {w} {h}: gRPC instead", flush=True)
        return False

    def spawn(self, j):
        if not self.a.direct:
            out = ssh(f"printf '{j}\\nq\\n' | {OFFICIAL}/MORAI_Spawn_Linux", 60)
            if f"SetVehiclePosition [{j}]" in out and "Error" not in out:
                return "official"
            print(f"official spawn {j} failed: gRPC instead: {out.strip()[-200:]}", flush=True)
        c = self.cps[j]
        self.sim.put(c[0], c[1], c[2] + 0.4, c[3])        # transform z = road + ~0.4 m (CP13 spawn: 28.68 vs 28.29)
        return "grpc"

    def ego(self):
        for k in range(60):                               # a simulator hiccup: retry for 30 s, then give up
            try:
                return self.sim.ego()
            except grpc.RpcError as e:
                err = e
                time.sleep(0.5)
        raise RuntimeError(f"simulator gRPC unreachable: {err}")

    def run_lap(self, k):
        seed = self.a.seed * 1000 + k
        name, shaded, meta = make_scene(seed)
        msc("pause")
        r = msc("load-scene", name + "/62", 90)
        if not r.startswith("ok"):
            raise RuntimeError(f"scene load {name}: {r}")
        lap = dict(lap=k, scene=name, seed=seed, weather=[], checkpoints=[], returns=[], scene_meta=meta)
        rows = {key: [] for key in ("ego_status", "collision", "signal", "gps")}
        clk = []                                          # (wall s, tau s): lap_video.py puts a screen recording on tau
        self.weather(lap, seed, 0, 0.0)
        played = threading.Event()                      # MSC confirms play 1-15 s late (10/26): record from the paused START on
        threading.Thread(target=lambda: (msc("play"), played.set()), daemon=True).start()
        tr, clock = self.track, self.clock
        tr.i, clock.last, passed = 0, None, 0
        tau_prev = clock.tau
        t_play, w_play, t_move, x0 = None, time.time(), None, None
        best_s = prog_s = 0.0
        prog_t, crash_t, off_t, grace = math.inf, -math.inf, None, math.inf
        end = t_end = None
        lt = lap_s = 0.0
        interrupted = False
        try:
            while end is None or tau < t_end + 1.0:          # 1 s past the end: the scorer's 10 Hz grid reaches it too
                t = time.time()
                e = self.ego()
                tau = clock.step(t, e["x"], e["y"], e["v"])
                clk.append((round(t, 3), round(tau, 3)))
                s, lat = tr.update(e["x"], e["y"])
                shade = shaded(e["x"], e["y"])
                ns = int(tau * 1e9)
                rows["ego_status"].append((ns, (e["x"], e["y"], e["yaw"], e["v"])))
                rows["collision"].append((ns, [(h, e["x"], e["y"]) for h in e["hit"]]))
                rows["gps"].append((ns, shade))                  # the scorer's lane-exempt zone = the GPS shadow
                if e["tl"]:
                    rows["signal"].append((ns, (e["tl"], e["color"])))
                x0 = x0 or (e["x"], e["y"])
                if t_move is None and math.hypot(e["x"] - x0[0], e["y"] - x0[1]) > self.cfg["time"]["move_m"]:
                    t_move = tau
                if t_play is None and (played.is_set() or t_move is not None):
                    t_play, prog_t, grace = tau, tau, tau + GRACE_S
                lt = tau - (t_move if t_move is not None else tau)
                best_s = max(best_s, s) if lat < 15.0 else best_s
                if passed < 14 and math.hypot(e["x"] - self.cps[passed + 1, 0], e["y"] - self.cps[passed + 1, 1]) <= CP_R:
                    passed += 1
                    lap["checkpoints"].append([passed, round(lt, 2)])
                    self.weather(lap, seed, passed, lt)
                if end is None:
                    if passed == 14 and s >= tr.s[-1] - FINISH_M:
                        end = "finish"
                    elif lt > LIMIT_S:
                        end = "time_limit"
                    elif t_play is not None and tau - t_play > START_S and best_s < START_FRAC * tr.s[-1]:
                        end = "start_fail"
                    t_end, lap_s = tau, lt
                # drive failures -> return to the last checkpoint passed
                if e["hit"]:
                    crash_t = tau
                if s > prog_s + PROGRESS_M:
                    prog_s, prog_t = s, tau
                elif e["tl"] and e["color"] & 1 and not e["color"] & 16:
                    # red time does not count as stuck: waits at C1256W000016 ran 53-61 s and were returned; resetting
                    # instead kept a car that let greens pass for 229 s from ever being returned (10/27 W1 laps)
                    prog_t += tau - tau_prev
                tau_prev = tau
                off = not shade and self.off_road(e["x"], e["y"], e["yaw"])
                off_t = (off_t if off_t is not None else tau) if off else None
                why = None
                if off_t is not None and tau - off_t >= OFF_ROAD_S:
                    why = "off_road"
                elif tau - prog_t >= STUCK_S or (crash_t > prog_t - 1.0 and tau - prog_t >= CRASH_STUCK_S):
                    why = "stuck_after_collision" if crash_t > prog_t - 1.0 else "stuck"
                elif passed < 14 and lat < 15.0 and s > tr.s[self.cp_i[passed + 1]] + SKIP_M:
                    why = f"missed_cp{passed + 1}"
                if end is None and why and t_move is not None and tau >= grace:
                    how = self.spawn(passed)
                    lap["returns"].append(dict(t=round(lt, 2), to=passed, reason=why, s=round(s, 1), how=how))
                    tr.i, prog_s, prog_t, crash_t, off_t = self.cp_i[passed], float(tr.s[self.cp_i[passed]]), tau, -math.inf, None
                    grace, clock.last = tau + GRACE_S, None
                self.status = (f"lap {k} {name}  t {lt:6.1f} s (wall {t - w_play:6.1f})  rho {clock.rho:.2f}\n"
                               f"CP {passed}/14  s {s:7.1f} m  lat {lat:4.1f} m  v {e['v'] * 3.6:5.1f} km/h  {'shadow ' if shade else ''}"
                               f"{'/'.join(map(str, lap['weather'][-1][1:3])) if lap['weather'] else ''}\n"
                               f"returns {len(lap['returns'])}" + (f"  last: {lap['returns'][-1]}" if lap["returns"] else ""))
                time.sleep(max(0.0, 1.0 / HZ - (time.time() - t)))
        except KeyboardInterrupt:                         # Ctrl-C: keep the lap so far, then stop
            interrupted, lap_s, end = True, (lap_s if end else lt), end or "interrupted"
        msc("pause")
        lap.update(end=end, lap_s=round(lap_s, 2), wall_s=round(time.time() - w_play, 1), rho_end=round(clock.rho, 3))
        lap["scorer"] = self.judge(rows)
        lap["return_penalty_s"] = RETURN_S * len(lap["returns"])
        lap["total_s"] = round(lap["lap_s"] + lap["scorer"].get("penalty_s", 0.0) + lap["return_penalty_s"], 2)
        with gzip.open(self.out / f"lap{k:03d}.rows.json.gz", "wt") as f:
            json.dump(rows, f)
        json.dump(clk, open(self.out / f"lap{k:03d}.clock.json", "w"))
        with open(self.out / "laps.jsonl", "a") as f:
            f.write(json.dumps(lap, ensure_ascii=False) + "\n")
        sc = lap["scorer"]
        self.history.append(f"lap {k} {name}: {end} {lap['lap_s']:.1f} s + penalties {sc.get('penalty_s', 0):.0f} "
                            f"+ returns {lap['return_penalty_s']:.0f} = {lap['total_s']:.1f} s  {sc.get('counts', {})}")
        print(self.history[-1], flush=True)
        if interrupted:
            raise KeyboardInterrupt

    def judge(self, rows):
        try:
            _, sm = self.score(self.episode_from_rows(rows, self.cfg, "eval_manager"), self.kmap, self.cfg)
        except Exception as e:                            # keep the lap record; rows are saved for a rescore
            return dict(error=repr(e))
        keep = ("finished", "duration_s", "penalty_s", "counts", "competition_failed", "checkpoints_reached", "start_ok")
        return {k: sm[k] for k in keep} | dict(events=[{k: v for k, v in ev.items() if k != "after_finish"}
                                                      for ev in sm["events"] if ev["type"] != "checkpoint"])


def serve(mgr, port):
    page = (f"<!doctype html><meta charset=utf-8><title>MORAI eval</title><style>body{{margin:0;background:#111;"
            f"color:#ddd;font:14px monospace}}img{{width:100%;max-width:1280px;display:block}}pre{{margin:0;padding:8px}}"
            f"</style><img src='{VIEW_URL}'><pre id=s></pre><script>setInterval(async()=>{{try{{"
            f"s.textContent=await (await fetch('/status')).text()}}catch(e){{}}}},1000)</script>").encode()

    class Page(BaseHTTPRequestHandler):
        def do_GET(self):
            body = ("\n".join([mgr.status, ""] + mgr.history[::-1]).encode() if self.path == "/status" else page)
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8" if self.path == "/status" else "text/html")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    threading.Thread(target=ThreadingHTTPServer(("127.0.0.1", port), Page).serve_forever, daemon=True).start()


def deploy(a):
    if SIM == "w1":
        return deploy_w1(a)
    if not a.direct:
        ok = ssh(f"cd {OFFICIAL} && sha256sum -c --quiet - <<'E' && echo verified\n{SPAWN_SHA}  MORAI_Spawn_Linux\n"
                 f"{WTC_SHA}  WeatherTimeChanger_Linux\nE\n")
        if "verified" not in ok:
            sys.exit(f"official programs missing or changed on {BC}:{OFFICIAL}: run tools/fetch_official_tools.sh (or --direct)")
    for src, dst in ((HERE / "scenario_variants.py", "morai-eval-scenario-variants.py"),
                     (MAP / "2026_molit_comp_global_path.txt", "morai-eval-global-path.txt"),
                     (HERE / "viewer.py", "morai-eval-viewer.py"), (HERE / "wtc.py", "morai-eval-wtc.py")):
        if BC != "local":
            subprocess.run(["scp", "-q", str(src), f"{BC}:{dst}"], check=True)
        elif not (Path.home() / dst).exists() or not filecmp.cmp(src, Path.home() / dst, shallow=False):
            tmp = Path.home() / f".{dst}.{os.getpid()}"       # managers of other worlds may be reading it: replace atomically
            shutil.copy(src, tmp); os.replace(tmp, Path.home() / dst)
    if not a.direct:                                      # a fresh program connected to the running simulator
        ssh("python3 ~/morai-eval-wtc.py down")
        print(ssh(f"python3 ~/morai-eval-wtc.py up {OFFICIAL}/WeatherTimeChanger_Linux", 60).strip(), flush=True)
    if not a.no_view:                                              # the Xwayland auth file is named anew each session
        ssh("kill -0 $(cat ~/morai-eval-viewer.pid 2>/dev/null) 2>/dev/null || { DISPLAY=:0 XAUTHORITY=$(ls /run/user/1000/"
            f".mutter-Xwaylandauth.* | head -1) setsid nohup python3 ~/morai-eval-viewer.py {HOST} 8090 {VIEW_FPS} {VIEW_WIDTH} > ~/morai-eval-viewer.log"
            " 2>&1 < /dev/null & echo $! > ~/morai-eval-viewer.pid; }")


WINVIEW_STOP = "powershell -NoProfile -ExecutionPolicy Bypass -File C:/morai-par/winview.ps1 -Stop"


def winview_down():
    subprocess.run(["pkill", "-f", "[s]sh -N -L 8090:127.0.0.1:8090 winpc-win"])
    subprocess.run(["ssh", "winpc-win", WINVIEW_STOP], capture_output=True)


def deploy_w1(a):
    for src, dst in ((HERE / "scenario_variants.py", "morai-eval-scenario-variants.py"),
                     (MAP / "2026_molit_comp_global_path.txt", "morai-eval-global-path.txt")):
        subprocess.run(["scp", "-q", str(src), f"{BC}:{dst}"], check=True)
    if a.no_view:
        return
    subprocess.run(["scp", "-q", str(HERE / "winview.ps1"), "winpc-win:C:/morai-par/winview.ps1"], check=True)
    winview_down()                                        # a fresh one, with this run's FPS and width
    print(subprocess.run(["python3", str(Path.home() / ".claude/skills/morai-use/scripts/remote_ps.py"), "--host", "winpc-win",
                          "--interactive", str(HERE / "winview.ps1"), f"Fps={VIEW_FPS}", f"Width={VIEW_WIDTH}"],
                         capture_output=True, text=True).stdout.strip(), flush=True)
    subprocess.Popen(["ssh", "-N", "-L", "8090:127.0.0.1:8090", "winpc-win"], stdin=subprocess.DEVNULL,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)    # the forward must not hold the caller's pipes


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name")
    ap.add_argument("--laps", type=int, default=0, help="0: until interrupted")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--weather", help="WEATHER:HOUR fixed for every lap (SUNNY|FOGGY, 11|13|15) instead of per section")
    ap.add_argument("--scorer", default=str(HERE.parents[3] / "Alpamayo-Cosmos" / "tools" / "scorer"))
    ap.add_argument("--out", default="~/morai-eval/runs")
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--no-view", action="store_true")
    ap.add_argument("--direct", action="store_true", help="returns and weather over gRPC instead of the official programs")
    ap.add_argument("--host", help="simulator host over ssh, or local (default: bc-250, or winpc with EVAL_SIM=w1)")
    ap.add_argument("--grpc", help="the world's gRPC host:port (default HOST:7789)")
    ap.add_argument("--launcher", help="the world's MoraiLauncher_Lin_Data (MSC link file, scene folder)")
    a = ap.parse_args()
    global BC, GRPC, LAUNCHER, SCEN
    BC, GRPC = a.host or BC, a.grpc or GRPC
    if a.launcher:
        LAUNCHER, SCEN = a.launcher, a.launcher + "/SaveFile/Scenario/R_KR_PR_K-city_2025"
    a.direct = a.direct or SIM == "w1" or not GRPC.endswith(":7789")   # the official programs only talk to 7789
    signal.signal(signal.SIGINT, signal.default_int_handler)   # background starts (eval3.sh) inherit SIGINT ignored
    mgr = Manager(a)
    deploy(a)
    serve(mgr, a.port)
    print(f"viewer: http://localhost:{a.port}  records: {mgr.out}", flush=True)
    k = 0
    try:
        while not a.laps or k < a.laps:
            mgr.run_lap(k)
            k += 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)     # a second Ctrl-C must not cut the clean-up short
        msc("pause")
        if not a.direct:
            ssh("python3 ~/morai-eval-wtc.py down")
        if not a.no_view and SIM == "w1":
            winview_down()
        elif not a.no_view:
            ssh("kill $(cat ~/morai-eval-viewer.pid) 2>/dev/null; true")


if __name__ == "__main__":
    main()
