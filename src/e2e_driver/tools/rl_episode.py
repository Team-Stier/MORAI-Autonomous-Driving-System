#!/usr/bin/env python3
"""RL episode manager (Alpamayo-Cosmos RL_PLAN.md 4.1): repeated section episodes from checkpoint START_CP to END_CP +
PAST_M, with the stack (run.sh or the pod's stack.sh) driving and noticing the resets itself, like eval_manager.py, whose
simulator, scene, weather and scorer parts it reuses.

    tools/rl_episode.sh NAME [--start-cp 9] [--end-cp 12] [--past-m 100] [--episodes N] [--seed S]
        [--host bc-250 | local] [--grpc HOST:PORT] [--launcher DIR] [--loc-restart CMD] [--direct]

Checkpoints are the official 10/7 table rows (checkpoints_20261007.csv, START = 0, as MORAI's Spawn program numbers
them). The 60 kph exemption (고주로) is CP10 s 1118.7 -> CP12 s 1741.7, so the default episode is CP9 (s 930.6) -> CP12
+ 100 m: the entry and the exit hand-over, where the limit applies again, are inside it.
Episode k (seed S*1000+k):
  - scene: a fresh clcol variant (scenario_variants.py, as an eval lap: signal phases, NPC traffic, obstacles drawn;
    --scene-preset brake: DAgger dynamic/tunnel movers that make the Ego stop)
  - pause -> load-scene -> Ego to START_CP with MORAI's Spawn program (gRPC SetTransform to the table pose with --direct
    or on a simulator whose gRPC port is not 7789: the program only talks to 7789) -> weather and time of the start
    section -> localization restart (LOC_RESTART) -> 4 s -> play
  - weather and time per checkpoint section, drawn from (seed, section) as in evaluation (rules §2-7-2)
  - ends: "goal" (route s >= s_goal = s(END_CP) + PAST_M), "slow" (past END_CP below 60 kph; not with --no-slow-end),
    a failure ("off_road": all wheels off the drivable area outside the shadow for the scorer's inoperable.min_s;
    "missed_cpN"; "stuck" / "stuck_after_collision" as in evaluation) or "time_limit" (LIMIT_S of simulator time after
    play: truncated)
  - recorded 1 s past the end, paused, scored by tools/scorer, rewards per decision (rl_reward.py), the privileged
    critic vector per decision (rl_critic.py, from the 20 Hz Ego GT and MORAI Object Info on --objinfo-port)
Records = the learner's spool (Alpamayo-Cosmos train/rl/episodes.py): OUT is the spool, NAME the actor.
  OUT/NAME/inflight.json   {version, episode}, written at every episode start (version: --lora-latest's "version")
  OUT/NAME/episodes.jsonl  one line per episode: end, time, return and its parts, scorer counts and events, weather
  OUT/NAME/epK/steps.jsonl one line per decision: the rl_reward record + critic_obs (rl_critic.FIELDS order) + req, out:
                           this world's engine dumps (--dump-dir: model_server --dump DIR, DIR/world{n}/, requests
                           carrying t_wall and world), moved into epK/ as K.req.bin / K.out.bin. Without --dump-dir the decisions are a
                           10 Hz grid of simulator time from play (baseline drives for the critic warm start).
  OUT/NAME/epK/done.json   {terminated, truncated, end}, written last
  OUT/NAME/epK/objects.json.gz   every 20 Hz frame: [tau, t_wall, Object Info records (id, type, x, y, z, heading deg,
                           length, width, height, as decode_objects gives them)] (the PPO critic's offline inputs)
--returns (the DAgger actor, user 10/28): a failure does not end the episode: as eval_manager's laps, off road
  (OFF_ROAD_S), stuck (STUCK_S, red time excluded; CRASH_STUCK_S after a collision) or a skipped checkpoint (SKIP_M)
  spawns the car at the last checkpoint passed and the lap goes on (GRACE_S before the next check); --return-hook runs
  after each. The episode records the returns (t, tau, t_wall, to, reason, s, how); steps.jsonl rows carry
  since_return_s (simulator s since the last return) once one happened: their 1.5 s history may cross the jump.
  OUT/NAME/epK/rows.json.gz, timeline.json   the 20 Hz Ego states the scorer read and the reward timeline (rl_reward.py
                           re-joins other decision times with it)
Hosts: --host bc-250 (ssh from the Mac, the two-PC eval set-up) or local (on the simulator host, e.g. the cloud pod). On
a pod world n: --world n --grpc 127.0.0.1:$((7788+n)) --objinfo-port $((7505+1000*(n-1))) --launcher
/root/inst/n/morai/MoraiLauncher_Lin_Data --direct.
"""
import argparse
import gzip
import json
import math
import os
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

import eval_manager as em
import rl_critic
import rl_reward

LIMIT_S = 150.0      # simulator s after play; CP9 -> CP12 + 100 m is 911 m (46.5 s median for the 623 m exemption alone)
# --red-cap LO HI (user 10/28): a red wait at a judged stop line ends after a time drawn from [LO, HI] s: the
# intersection gets its phase with the route's green (SetIntersectionPhase over Infrastructure gRPC restarts that phase,
# then the normal cycle; morai-parallel-collect r2_lights). The cycle itself is fixed in the map (MGeo), so a scene can
# only set the arrival phase; this keeps approach, stop, start and yellow while a wait never fills an episode.
# route s of the judged stop line -> (intersection, phase with the route's green, MORAI status bit of that green)
RED_LIGHTS = {208.5: ("IntTL9", 1, 16), 315.9: ("IntTL7", 0, 16), 436.8: ("IntTL3", 2, 16), 609.9: ("IntTL1", 3, 32)}


def set_phase(ch, intersection, phase):
    """Infrastructure/SetIntersectionPhase(IntersectionPhase{1 id: ObjectIdentifier{1 value}, 2 phase}), raw protobuf."""
    ld = lambda f, p: bytes([f << 3 | 2, len(p)]) + p
    return ch.unary_unary("/morai_sim_api.infrastructure.Infrastructure/SetIntersectionPhase", request_serializer=lambda x: x,
                          response_deserializer=lambda x: x)(ld(1, ld(1, intersection.encode())) + bytes([0x10, phase]), timeout=10)
HERE = Path(__file__).resolve().parent


class Episodes(em.Manager):
    def __init__(self, a):
        super().__init__(a)
        self.limit_s, self.min_off = a.limit_s, self.cfg["inoperable"]["min_s"]
        self.s_exit = float(self.track.s[self.cp_i[a.end_cp]])
        self.s_goal = self.s_exit + a.past_m
        self.lanes = rl_critic.Lanes(self.cfg)
        self.objects = objinfo(a.objinfo_port) if a.objinfo_port else (lambda: [])

    def reset(self, k, seed):
        """pause -> load-scene -> Spawn START_CP -> start section's weather -> localization restart -> 4 s (-> play)."""
        name, shaded, meta = em.make_scene(seed, self.a.scene_preset)
        em.msc("pause")
        r = em.msc("load-scene", name + "/62", 90)
        if not r.startswith("ok"):
            raise RuntimeError(f"scene load {name}: {r}")
        ep = dict(episode=k, scene=name, seed=seed, start_cp=self.a.start_cp, end_cp=self.a.end_cp,
                  s_goal=round(self.s_goal, 1), weather=[], checkpoints=[], scene_meta=meta)
        (self.out / f"ep{k:05d}").mkdir(exist_ok=True)
        latest = Path(self.a.lora_latest).expanduser() if self.a.lora_latest else None
        version = json.load(open(latest)).get("version", 0) if latest and latest.exists() else 0
        write_json(self.out / "inflight.json", dict(version=version, episode=f"ep{k:05d}"))
        ep["spawn"] = self.spawn(self.a.start_cp)
        self.start_at = (self.cp_i[self.a.start_cp], self.a.start_cp)
        if self.a.spawn_before:                           # popout: the Ego LO-HI m short of the segment's mover, goal 25 m past it
            import scenario_variants as sv
            s_cp = float(self.track.s[self.cp_i[self.a.start_cp]])
            s_obj = min(a["s"] for a in meta["actors"] if a["zone"].endswith("_dynamic") and a["s"] > s_cp)
            s_sp = s_obj - float(np.random.default_rng(seed).uniform(*self.a.spawn_before))
            x, y, z, h = sv.Route(str(em.MAP / "2026_molit_comp_global_path.txt")).at(s_sp, 0.0)
            self.sim.put(x, y, z + 0.4, h)
            j = int(np.argmin(np.hypot(*(self.kmap.path - (x, y)).T)))
            self.start_at = (j, max(c for c in range(15) if self.track.s[self.cp_i[c]] <= s_sp))
            self.s_goal = s_obj + 25.0
            ep.update(spawn="grpc_before", spawn_s=round(s_sp, 1), s_goal=round(self.s_goal, 1))
        self.weather(ep, seed, self.a.start_cp, 0.0)
        subprocess.run(["bash", "-c", self.a.loc_restart], capture_output=True, timeout=60)
        time.sleep(4.0)
        return ep, shaded

    def run_episode(self, k):
        seed = self.a.seed * 1000 + k
        ep, shaded = self.reset(k, seed)
        rows = {key: [] for key in ("ego_status", "collision", "signal", "gps")}
        clk = []
        played = threading.Event()                      # MSC confirms play 1-15 s late: record from the paused start on
        threading.Thread(target=lambda: (em.msc("play"), played.set()), daemon=True).start()
        tr, clock, cps = self.track, self.clock, self.cps
        self.critic = rl_critic.Critic(self.kmap, self.cfg, self.lanes, self.s_exit, self.s_goal)
        (tr.i, passed), clock.last = self.start_at, None
        t_play = t_move = x0 = None
        prog_s, prog_t, crash_t, off_t, grace = 0.0, math.inf, -math.inf, None, math.inf
        end = t_end = None
        waits, rng, ep["red_cap"], ep["returns"], objt, tau_prev = {}, np.random.default_rng(seed), [], [], [], None
        w0 = time.time()
        while end is None or tau < t_end + 1.0:          # 1 s past the end: the scorer's 10 Hz grid reaches it too
            t = time.time()
            e = self.ego()
            tau = clock.step(t, e["x"], e["y"], e["v"])
            clk.append((round(t, 3), round(tau, 3)))
            s, lat = tr.update(e["x"], e["y"])
            shade = shaded(e["x"], e["y"])
            objs = self.objects()
            self.critic.step(tau, e["x"], e["y"], e["yaw"], e["v"], s, tr.i, objs, shade)
            objt.append([round(tau, 3), round(t, 3), [[o[0], o[1], *(round(float(v), 3) for v in o[2:])] for o in objs]])
            ns = int(tau * 1e9)
            rows["ego_status"].append((ns, (e["x"], e["y"], e["yaw"], e["v"])))
            rows["collision"].append((ns, [(h, e["x"], e["y"]) for h in e["hit"]]))
            rows["gps"].append((ns, shade))
            if e["tl"]:
                rows["signal"].append((ns, (e["tl"], e["color"])))
            x0 = x0 or (e["x"], e["y"])
            if t_move is None and math.hypot(e["x"] - x0[0], e["y"] - x0[1]) > self.cfg["time"]["move_m"]:
                t_move = tau
            if t_play is None and (played.is_set() or t_move is not None):
                t_play, prog_s, prog_t, grace = tau, s, tau, tau + em.GRACE_S
            if passed < 14 and math.hypot(e["x"] - cps[passed + 1, 0], e["y"] - cps[passed + 1, 1]) <= em.CP_R:
                passed += 1
                ep["checkpoints"].append([passed, round(tau - t_play, 2) if t_play is not None else 0.0])
                self.weather(ep, seed, passed, tau - (t_play or tau))
            if self.a.red_cap and t_play is not None:   # cap red waits at the judged stop lines
                ahead = self.a.red_cap[1] < 0             # --red-cap -1 -1: the route's green on approach, no stop
                for s_stop, (ic, ph, bit) in RED_LIGHTS.items():
                    if not s_stop - (60.0 if ahead else 40.0) <= s <= s_stop + 2.0:
                        continue
                    col = e["color"] if e["color"] is not None else -1
                    if (abs(e["v"]) >= 0.5 and not ahead) or (col >= 0 and col & bit):
                        waits.pop(s_stop, None)   # moving, or the route's green is on; no light reading (-1) counts as
                        continue                  # waiting (10/28: -1 & 32 read as green kept a CP6 wait from ever ending)
                    w = waits.setdefault(s_stop, [tau, float(rng.uniform(*self.a.red_cap)), None])
                    if tau - w[0] >= w[1] and (w[2] is None or tau - w[2] > 3.0):
                        try:
                            set_phase(self.sim.ch, ic, ph)
                        except Exception as err:   # grpc.RpcError: the wait just goes on
                            print(f"red cap {ic}: {err}", flush=True)
                        w[2] = tau
                        ep["red_cap"].append([s_stop, round(tau - w[0], 1), round(tau - (t_play or tau), 1)])
            if e["hit"]:
                crash_t = tau
                if self.a.end_on_hit and end is None and t_play is not None and tau - t_play > 1.0 and any(str(h) != "-100" for h in e["hit"]):   # popout: the next scene at once; 10/29: a phantom "-100" contact sticks from the spawn on after some batches, the car drives on
                    # (a contact in the first second is the spawn settling; an episode with no decisions broke finish())
                    end, t_end = "collision", tau
            if s > prog_s + em.PROGRESS_M:
                prog_s, prog_t = s, tau
            elif self.a.returns and tau_prev is not None and e["tl"] and e["color"] & 1 and not e["color"] & 16:
                prog_t += tau - tau_prev                  # as eval_manager: red time does not count as stuck
            tau_prev = tau
            off = not shade and self.off_road(e["x"], e["y"], e["yaw"])
            off_t = (off_t if off_t is not None else tau) if off else None
            why = None                                    # --returns: eval_manager's return rules, then on from the last CP
            if self.a.returns and end is None and t_play is not None and tau >= grace:
                if off_t is not None and tau - off_t >= em.OFF_ROAD_S:
                    why = "off_road"
                elif tau - prog_t >= em.STUCK_S or (crash_t > prog_t - 1.0 and tau - prog_t >= em.CRASH_STUCK_S):
                    why = "stuck_after_collision" if crash_t > prog_t - 1.0 else "stuck"
                elif passed < 14 and lat < 15.0 and s > tr.s[self.cp_i[passed + 1]] + em.SKIP_M:
                    why = f"missed_cp{passed + 1}"
            if why:
                how = self.spawn(passed)
                ep["returns"].append(dict(t=round(tau - t_play, 2), tau=round(tau, 3), t_wall=round(t, 3), to=passed,
                                          reason=why, s=round(s, 1), how=how))
                tr.i, prog_s, prog_t, crash_t, off_t = self.cp_i[passed], float(tr.s[self.cp_i[passed]]), tau, -math.inf, None
                grace, clock.last = tau + em.GRACE_S, None
                if self.a.return_hook:                    # e.g. the shadow's planners must not see the jump
                    threading.Thread(target=lambda: subprocess.run(["bash", "-c", self.a.return_hook], capture_output=True,
                                                                   timeout=120), daemon=True).start()
            if end is None and t_play is not None:
                if (passed >= self.a.end_cp or self.a.spawn_before) and lat < 15.0 and s >= self.s_goal:
                    end = "goal"
                elif not self.a.no_slow_end and passed >= self.a.end_cp and abs(e["v"]) * 3.6 < self.cfg["speed"]["limit_kph"]:
                    end = "slow"
                elif not self.a.returns and tau >= grace and off_t is not None and tau - off_t >= self.min_off:
                    end = "off_road"
                elif not self.a.returns and tau >= grace and (tau - prog_t >= em.STUCK_S or (crash_t > prog_t - 1.0 and tau - prog_t >= em.CRASH_STUCK_S)):
                    end = "stuck_after_collision" if crash_t > prog_t - 1.0 else "stuck"
                elif not self.a.returns and passed < self.a.end_cp and lat < 15.0 and s > tr.s[self.cp_i[passed + 1]] + em.SKIP_M:
                    end = f"missed_cp{passed + 1}"
                elif tau - t_play > self.limit_s:
                    end = "time_limit"
                t_end = tau
            self.status = (f"episode {k} {ep['scene']}  t {tau - (t_play or tau):6.1f} s  rho {clock.rho:.2f}  CP {passed}  "
                           f"s {s:7.1f}/{self.s_goal:.0f} m  v {e['v'] * 3.6:5.1f} km/h  {end or ''}")
            time.sleep(max(0.0, 1.0 / em.HZ - (time.time() - t)))
        em.msc("pause")
        if self.a.end_hook:                               # e.g. the episode's rosbag closes
            subprocess.run(["bash", "-c", self.a.end_hook], capture_output=True, timeout=60)
        self.finish(ep, rows, clk, end, t_play, t_end, time.time() - w0, objt)

    def finish(self, ep, rows, clk, end, t_play, t_end, wall_s, objt=()):
        k = ep["episode"]
        d = self.out / f"ep{k:05d}"
        with gzip.open(d / "objects.json.gz", "wt") as f:   # every 20 Hz frame: Object Info as decode_objects gives it
            json.dump(dict(fields=["tau", "t_wall", "objects"],
                           object_fields=["id", "type", "x", "y", "z", "heading_deg", "length", "width", "height"],
                           frames=objt), f)
        labels, sm = self.score(self.episode_from_rows(rows, self.cfg, "rl_episode"), self.kmap, self.cfg)
        t0 = rows["ego_status"][0][0] / 1e9               # scorer frame t = 0
        if self.a.end_hook:   # a recorded episode (popout_box.sh): the scorer labels and weather on the host clock for
            W, T = np.asarray(clk, float).T               # tools/step0 (bag stamps are host time; t stays tau)
            labels.assign(bag_time_ns=(np.interp(labels.t.to_numpy(float) + t0, T, W) * 1e9).astype(np.int64)).to_parquet(d / "labels.parquet")
            with open(d / "env.log", "w") as f:
                for _, w, h, t_rel, _ in ep["weather"]:
                    f.write(f"{float(np.interp((t_play or T[0]) + t_rel, T, W)) - (0.0 if t_rel else 5.0):.3f} {w} {h}\n")
        tl = rl_reward.timeline(labels, sm, self.cfg, t0, end, t_end, self.s_goal, clk)
        tl.update(episode=f"{self.a.name}/ep{k:05d}", reward_args=dict(v_ref=self.a.v_ref, fail_s=self.a.fail_s))
        W, T = np.asarray(clk, float).T
        dec, outside = self.decisions(float(np.interp(t_play, T, W)), float(np.interp(t_end, T, W))) if self.a.dump_dir else ([], [])
        for src in outside:                               # this world's decisions while paused or past the end: kept apart
            (d / "outside").mkdir(exist_ok=True)
            shutil.move(src, d / "outside" / src.name)
        t_dec = np.interp([w for w, _, _ in dec], W, T) if self.a.dump_dir else np.arange(t_play, t_end, 0.1)
        recs = rl_reward.rewards(tl, t_dec, **tl["reward_args"])
        if not recs:                                      # ended at once (no 0.1 s step): nothing to keep
            print(f"episode {k} {ep['scene']}: {end} 0.0 s, no decisions (skipped)", flush=True)
            return
        prev = [(0.0, 0.0)] + [first_step(q) for _, _, q in dec[:len(recs) - 1]] if dec else [(0.0, 0.0)] * len(recs)
        obs = self.critic.at([r["t_sim"] for r in recs], prev)
        with gzip.open(d / "rows.json.gz", "wt") as f:
            json.dump(rows, f)
        json.dump(tl, open(d / "timeline.json", "w"))
        with open(d / "steps.jsonl", "w") as f:
            for j, (r, o) in enumerate(zip(recs, obs)):
                step = dict(r, episode=tl["episode"], critic_obs=o)
                back = [r["t_sim"] - x["tau"] for x in ep.get("returns", []) if x["tau"] <= r["t_sim"]]
                if back:                                  # history crosses a return's jump for 1.5 s (labels: none)
                    step["since_return_s"] = round(min(back), 3)
                if dec:                                   # this world's engine dumps move into the episode
                    for kind, src in (("req", dec[j][1]), ("out", dec[j][2])):
                        step[kind] = f"{j:05d}.{kind}.bin"
                        shutil.move(src, d / step[kind])
                f.write(json.dumps(step) + "\n")
        parts = {p: round(sum(r[p] for r in recs), 2) for p in ("r", "r_time", "r_penalty", "r_shape", "r_terminal")}
        write_json(d / "done.json", dict(terminated=recs[-1]["terminated"], truncated=recs[-1]["truncated"], end=end))
        ep.update(end=end, t_sim=round(t_end - t_play, 2), wall_s=round(wall_s, 1), rho_end=round(self.clock.rho, 3),
                  decisions=len(recs), engine_decisions=bool(dec), terminated=recs[-1]["terminated"],
                  truncated=recs[-1]["truncated"], **parts,
                  scorer={key: sm[key] for key in ("penalty_s", "counts", "competition_failed", "checkpoints_reached")}
                  | dict(events=[ev for ev in sm["events"] if ev["type"] != "checkpoint"]))
        with open(self.out / "episodes.jsonl", "a") as f:
            f.write(json.dumps(ep, ensure_ascii=False) + "\n")
        self.history.append(f"episode {k} {ep['scene']}: {end} {ep['t_sim']:.1f} s, returns {len(ep.get('returns', []))} "
                            f"{[x['reason'] for x in ep.get('returns', [])]}, penalties {sm['penalty_s']:.0f}, "
                            f"return {parts['r']:.1f}, {len(recs)} decisions {sm['counts']}")
        print(self.history[-1], flush=True)

    def decisions(self, w0, w1):
        """-> ([(t_wall, req, out)] of this world's engine decisions with t_wall in [w0, w1] (host clock), oldest first,
        [this world's other dump files]): the next episode has not started, so every dump of this world is one or the
        other."""
        time.sleep(1.0)                                   # the last responses land ~0.1 s after the pause
        inside, outside = [], []
        for p in Path(self.a.dump_dir).expanduser().glob(f"world{self.a.world}/req_*.bin" if self.a.world else "*/req_*.bin"):
            h = head(p)
            if self.a.world and h.get("world") != self.a.world:
                continue
            q = p.with_name(p.name.replace("req_", "out_"))
            if w0 <= h.get("t_wall", -1.0) <= w1 and q.exists():
                inside.append((h["t_wall"], p, q))
            else:
                outside += [p] + ([q] if q.exists() else [])
        return sorted(inside), outside


def write_json(path, obj):
    tmp = Path(str(path) + ".tmp")
    tmp.write_text(json.dumps(obj))
    os.replace(tmp, path)


def head(path):
    """the JSON header of a wire frame (e2e_driver.wire): a request's scalars without reading its images."""
    with open(path, "rb") as f:
        return json.loads(f.read(struct.unpack(">I", f.read(4))[0]))["obj"]


def first_step(path):
    """(accel[0], kappa[0]) of an engine response dump."""
    sys.path.insert(0, str(HERE.parent / "src"))
    from e2e_driver import wire
    o = wire.unpack(Path(path).read_bytes())
    return float(o["accel"][0]), float(o["kappa"][0])


def objinfo(port):
    """-> objects(): the MORAI Object Info records of the last 0.3 s, newest per id (a frame with more than 20 objects
    comes in several packets), from a receiver thread on UDP port (7505 + world offset)."""
    sys.path.insert(0, str(HERE.parents[1] / "visualizer" / "scripts"))
    from object_info_udp import decode_objects
    seen = {}

    def run():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", port))
        while True:
            pkt = sock.recv(65535)
            try:
                t = time.time()
                seen.update({o[0]: (t, o) for o in decode_objects(pkt)})
            except ValueError:
                pass

    threading.Thread(target=run, daemon=True).start()
    return lambda: [o for t, o in list(seen.values()) if t > time.time() - 0.3]


def put(host, src, dst):
    subprocess.run(["cp", str(src), str(Path(dst).expanduser())] if host == "local" else ["scp", "-q", str(src), f"{host}:{dst}"],
                   check=True)


def deploy(a):
    """the scene tools on the simulator host (the official programs: tools/fetch_official_tools.sh, as for eval.sh)."""
    if not a.direct:
        ok = em.ssh(f"cd {em.OFFICIAL} && sha256sum -c --quiet - <<'E' && echo verified\n{em.SPAWN_SHA}  MORAI_Spawn_Linux\n"
                    f"{em.WTC_SHA}  WeatherTimeChanger_Linux\nE\n")
        if "verified" not in ok:
            raise SystemExit(f"official programs missing or changed on {a.host}:{em.OFFICIAL}: tools/fetch_official_tools.sh (or --direct)")
    for src, dst in ((HERE / "scenario_variants.py", "~/morai-eval-scenario-variants.py"),
                     (em.MAP / "2026_molit_comp_global_path.txt", "~/morai-eval-global-path.txt"),
                     (HERE / "msc.py", "~/morai-eval-msc.py"), (HERE / "wtc.py", "~/morai-eval-wtc.py")):
        put(a.host, src, dst)
    if not a.direct:
        em.ssh("python3 ~/morai-eval-wtc.py down")
        print(em.ssh(f"python3 ~/morai-eval-wtc.py up {em.OFFICIAL}/WeatherTimeChanger_Linux", 60).strip(), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name")
    ap.add_argument("--start-cp", type=int, default=9)
    ap.add_argument("--end-cp", type=int, default=12)
    ap.add_argument("--past-m", type=float, default=100.0)
    ap.add_argument("--episodes", type=int, default=0, help="0: until interrupted")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--limit-s", type=float, default=LIMIT_S)
    ap.add_argument("--v-ref", type=float, default=80 / 3.6, help="m/s of the progress potential Phi(s) = s / v_ref")
    ap.add_argument("--fail-s", type=float, default=100.0, help="terminal penalty of a failure, s")
    ap.add_argument("--world", type=int, default=0, help="this world's number in the engine requests (0: take all)")
    ap.add_argument("--dump-dir", help="the engine's --dump DIR (decisions with req/out); none: a 10 Hz grid")
    ap.add_argument("--objinfo-port", type=int, default=0, help="MORAI Object Info UDP port (0: no cars in critic_obs)")
    ap.add_argument("--lora-latest", help="the learner's <out>/lora/latest.json (inflight.json version)")
    ap.add_argument("--host", default=em.BC, help="simulator host over ssh, or local")
    ap.add_argument("--grpc", default=em.GRPC)
    ap.add_argument("--launcher", default=em.LAUNCHER)
    ap.add_argument("--loc-restart", default="ssh winpc-wsl \"cd morai-eval/ws/MORAI-Autonomous-Driving-System/src/e2e_driver/"
                    "tools/winpc && ./chroot.sh ctl 'source devel/setup.bash; rosnode kill /localization_node'\"",
                    help="shell command that restarts the stack's localization (respawns from the next GPS fix)")
    ap.add_argument("--weather", help="WEATHER:HOUR fixed for every episode instead of per section")
    ap.add_argument("--scorer", default=str(HERE.parents[3] / "Alpamayo-Cosmos" / "tools" / "scorer"))
    ap.add_argument("--out", default="~/morai-eval/rl", help="the learner spool (NAME = actor)")
    ap.add_argument("--port", type=int, default=8092)
    ap.add_argument("--direct", action="store_true", help="Spawn and weather over gRPC instead of the official programs")
    ap.add_argument("--no-slow-end", action="store_true", help="end at the goal only, not \"slow\" past END_CP (a segment "
                    "goal short of the next light: the dynamic zone's CP5 + 90 m stops before the CP6 light)")
    ap.add_argument("--scene-preset", default="", help="scenario_variants.py SCENE_PRESET (brake: DAgger dynamic/tunnel scenes)")
    ap.add_argument("--red-cap", type=float, nargs=2, metavar=("LO", "HI"), help="red waits end after a time drawn from [LO, HI] s "
                    "(-1 -1: the route's green once within 60 m)")
    ap.add_argument("--spawn-before", type=float, nargs=2, metavar=("LO", "HI"), help="spawn LO-HI m short of the mover, goal 25 m past it")
    ap.add_argument("--end-on-hit", action="store_true", help="a collision ends the episode")
    ap.add_argument("--end-hook", help="shell command run at each episode's end (before scoring); also writes the episode's "
                    "labels.parquet (host-clock bag_time_ns) and env.log for tools/step0")
    ap.add_argument("--returns", action="store_true", help="a failure returns to the last checkpoint passed (eval_manager's "
                    "rules) and the lap goes on; the episode ends at the goal or --limit-s")
    ap.add_argument("--return-hook", help="shell command run (in the background) after each return")
    ap.set_defaults(sync=None, tick_ms=100, serial=False)   # eval_manager.Manager's sync-mode arguments: free-running here
    a = ap.parse_args()
    assert 0 <= a.start_cp < a.end_cp <= 14, (a.start_cp, a.end_cp)
    em.BC, em.GRPC, em.HOST = a.host, a.grpc, a.grpc.rsplit(":", 1)[0]
    em.LAUNCHER, em.SCEN = a.launcher, a.launcher + "/SaveFile/Scenario/R_KR_PR_K-city_2025"
    a.direct = a.direct or not a.grpc.endswith(":7789")   # the official programs only talk to 7789
    mgr = Episodes(a)
    deploy(a)
    em.serve(mgr, a.port)
    print(f"status: http://localhost:{a.port}/status  records: {mgr.out}  s_goal {mgr.s_goal:.1f} m", flush=True)
    k = 0
    try:
        while not a.episodes or k < a.episodes:
            mgr.run_episode(k)
            k += 1
    finally:
        em.msc("pause")
        if not a.direct:
            em.ssh("python3 ~/morai-eval-wtc.py down")


if __name__ == "__main__":
    main()
