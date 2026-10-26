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
  - scene: a fresh clcol variant (scenario_variants.py, as an eval lap: signal phases, NPC traffic, obstacles drawn)
  - pause -> load-scene -> Ego to START_CP with MORAI's Spawn program (gRPC SetTransform to the table pose with --direct
    or on a simulator whose gRPC port is not 7789: the program only talks to 7789) -> weather and time of the start
    section -> localization restart (LOC_RESTART) -> 4 s -> play
  - weather and time per checkpoint section, drawn from (seed, section) as in evaluation (rules §2-7-2)
  - ends: "goal" (route s >= s_goal = s(END_CP) + PAST_M), "slow" (past END_CP below 60 kph), a failure ("off_road":
    all wheels off the drivable area outside the shadow for the scorer's inoperable.min_s; "missed_cpN"; "stuck" /
    "stuck_after_collision" as in evaluation) or "time_limit" (LIMIT_S of simulator time after play: truncated)
  - recorded 1 s past the end, paused, scored by tools/scorer, rewards per decision (rl_reward.py)
Records (OUT/NAME/): episodes.jsonl (one line per episode: end, time, return and its parts, scorer counts and events,
weather), epK.rows.json.gz (20 Hz Ego states the scorer read, on simulator time), epK.timeline.json (penalties, route s,
wall<->simulator clock: rl_reward.py re-joins the actor's decision times with it), epK.rewards.jsonl (learner records on
a 10 Hz grid of simulator time from play).
Hosts: --host bc-250 (ssh from the Mac, the two-PC eval set-up) or local (on the simulator host, e.g. the cloud pod). On
a pod world n: --grpc 127.0.0.1:$((7788+n)) --launcher /root/inst/n/morai/MoraiLauncher_Lin_Data --direct.
"""
import argparse
import gzip
import json
import math
import subprocess
import threading
import time
from pathlib import Path

import numpy as np

import eval_manager as em
import rl_reward

LIMIT_S = 150.0      # simulator s after play; CP9 -> CP12 + 100 m is 911 m (46.5 s median for the 623 m exemption alone)
HERE = Path(__file__).resolve().parent


class Episodes(em.Manager):
    def __init__(self, a):
        super().__init__(a)
        self.limit_s, self.min_off = a.limit_s, self.cfg["inoperable"]["min_s"]
        self.s_goal = float(self.track.s[self.cp_i[a.end_cp]]) + a.past_m

    def reset(self, k, seed):
        """pause -> load-scene -> Spawn START_CP -> start section's weather -> localization restart -> 4 s (-> play)."""
        name, shaded, meta = em.make_scene(seed)
        em.msc("pause")
        r = em.msc("load-scene", name + "/62", 90)
        if not r.startswith("ok"):
            raise RuntimeError(f"scene load {name}: {r}")
        ep = dict(episode=k, scene=name, seed=seed, start_cp=self.a.start_cp, end_cp=self.a.end_cp,
                  s_goal=round(self.s_goal, 1), weather=[], checkpoints=[], scene_meta=meta)
        ep["spawn"] = self.spawn(self.a.start_cp)
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
        tr.i, clock.last, passed = self.cp_i[self.a.start_cp], None, self.a.start_cp
        t_play = t_move = x0 = None
        prog_s, prog_t, crash_t, off_t, grace = 0.0, math.inf, -math.inf, None, math.inf
        end = t_end = None
        w0 = time.time()
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
            if e["hit"]:
                crash_t = tau
            if s > prog_s + em.PROGRESS_M:
                prog_s, prog_t = s, tau
            off = not shade and self.off_road(e["x"], e["y"], e["yaw"])
            off_t = (off_t if off_t is not None else tau) if off else None
            if end is None and t_play is not None:
                if passed >= self.a.end_cp and lat < 15.0 and s >= self.s_goal:
                    end = "goal"
                elif passed >= self.a.end_cp and abs(e["v"]) * 3.6 < self.cfg["speed"]["limit_kph"]:
                    end = "slow"
                elif tau >= grace and off_t is not None and tau - off_t >= self.min_off:
                    end = "off_road"
                elif tau >= grace and (tau - prog_t >= em.STUCK_S or (crash_t > prog_t - 1.0 and tau - prog_t >= em.CRASH_STUCK_S)):
                    end = "stuck_after_collision" if crash_t > prog_t - 1.0 else "stuck"
                elif passed < self.a.end_cp and lat < 15.0 and s > tr.s[self.cp_i[passed + 1]] + em.SKIP_M:
                    end = f"missed_cp{passed + 1}"
                elif tau - t_play > self.limit_s:
                    end = "time_limit"
                t_end = tau
            self.status = (f"episode {k} {ep['scene']}  t {tau - (t_play or tau):6.1f} s  rho {clock.rho:.2f}  CP {passed}  "
                           f"s {s:7.1f}/{self.s_goal:.0f} m  v {e['v'] * 3.6:5.1f} km/h  {end or ''}")
            time.sleep(max(0.0, 1.0 / em.HZ - (time.time() - t)))
        em.msc("pause")
        self.finish(ep, rows, clk, end, t_play, t_end, time.time() - w0)

    def finish(self, ep, rows, clk, end, t_play, t_end, wall_s):
        k = ep["episode"]
        labels, sm = self.score(self.episode_from_rows(rows, self.cfg, "rl_episode"), self.kmap, self.cfg)
        t0 = rows["ego_status"][0][0] / 1e9               # scorer frame t = 0
        tl = rl_reward.timeline(labels, sm, self.cfg, t0, end, t_end, self.s_goal, clk)
        tl.update(episode=f"{self.a.name}/{k}", reward_args=dict(v_ref=self.a.v_ref, fail_s=self.a.fail_s))
        recs = rl_reward.rewards(tl, np.arange(t_play, t_end, 0.1), **tl["reward_args"])
        with gzip.open(self.out / f"ep{k:04d}.rows.json.gz", "wt") as f:
            json.dump(rows, f)
        json.dump(tl, open(self.out / f"ep{k:04d}.timeline.json", "w"))
        with open(self.out / f"ep{k:04d}.rewards.jsonl", "w") as f:
            f.writelines(json.dumps(dict(r, episode=tl["episode"])) + "\n" for r in recs)
        parts = {p: round(sum(r[p] for r in recs), 2) for p in ("r", "r_time", "r_penalty", "r_shape", "r_terminal")}
        ep.update(end=end, t_sim=round(t_end - t_play, 2), wall_s=round(wall_s, 1), rho_end=round(self.clock.rho, 3),
                  decisions=len(recs), terminated=recs[-1]["terminated"], truncated=recs[-1]["truncated"], **parts,
                  scorer={key: sm[key] for key in ("penalty_s", "counts", "competition_failed", "checkpoints_reached")}
                  | dict(events=[ev for ev in sm["events"] if ev["type"] != "checkpoint"]))
        with open(self.out / "episodes.jsonl", "a") as f:
            f.write(json.dumps(ep, ensure_ascii=False) + "\n")
        self.history.append(f"episode {k} {ep['scene']}: {end} {ep['t_sim']:.1f} s, penalties {sm['penalty_s']:.0f}, "
                            f"return {parts['r']:.1f} {sm['counts']}")
        print(self.history[-1], flush=True)


def put(host, src, dst):
    subprocess.run(["cp", str(src), str(Path(dst).expanduser())] if host == "local" else ["scp", "-q", str(src), f"{host}:{dst}"],
                   check=True)


def deploy(a):
    """the scene tools on the simulator host (the official programs: tools/fetch_official_tools.sh, as for eval.sh)."""
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
    ap.add_argument("--host", default=em.BC, help="simulator host over ssh, or local")
    ap.add_argument("--grpc", default=em.GRPC)
    ap.add_argument("--launcher", default=em.LAUNCHER)
    ap.add_argument("--loc-restart", default="ssh winpc-wsl \"cd morai-eval/ws/MORAI-Autonomous-Driving-System/src/e2e_driver/"
                    "tools/winpc && ./chroot.sh ctl 'source devel/setup.bash; rosnode kill /localization_node'\"",
                    help="shell command that restarts the stack's localization (respawns from the next GPS fix)")
    ap.add_argument("--weather", help="WEATHER:HOUR fixed for every episode instead of per section")
    ap.add_argument("--scorer", default=str(HERE.parents[3] / "Alpamayo-Cosmos" / "tools" / "scorer"))
    ap.add_argument("--out", default="~/morai-eval/rl")
    ap.add_argument("--port", type=int, default=8092)
    ap.add_argument("--direct", action="store_true", help="Spawn and weather over gRPC instead of the official programs")
    a = ap.parse_args()
    assert 0 <= a.start_cp < a.end_cp <= 14, (a.start_cp, a.end_cp)
    em.BC, em.GRPC, em.HOST = a.host, a.grpc, a.grpc.rsplit(":", 1)[0]
    em.LAUNCHER, em.SCEN = a.launcher, a.launcher + "/SaveFile/Scenario/R_KR_PR_K-city_2025"
    a.direct = a.direct or not a.grpc.endswith(":7789")   # the official programs only talk to 7789
    a.no_view = True
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
