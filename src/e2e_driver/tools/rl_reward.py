#!/usr/bin/env python3
"""PPO rewards per decision for one RL episode (Alpamayo-Cosmos RL_PLAN.md 4.2), from tools/scorer's judgments.

Rewards are seconds of the competition score, negative = lost. Per decision k (interval (t_k, t_k+1], the last one ending
at the episode end): r = r_time + r_penalty + r_shape + r_terminal.
- r_time: -(t_k+1 - t_k), the decision's share of the drive time (-0.1 s at 10 Hz of simulator time).
- r_penalty: minus the scorer's penalties falling in the interval: collision 15 at its start, signal 30 at the stop line,
  speed 15 at once and 15 more per further 3 s over the limit, lane 5 per 3 s of contact, each at the frame where the
  scorer's count reaches it, so they add up to the scorer's penalty_s (exempt span, shadow zone: the scorer's rules).
- r_shape: Phi(s_k+1) - Phi(s_k), Phi(s) = s / v_ref (route progress, potential-based with gamma 1; phi and phi_next are
  recorded so the learner can redo it for its own gamma). At a termination Phi = Phi(s_goal), a constant: the shaping
  then sums to Phi(s_goal) - Phi(s_0) in every terminated episode and cannot change which policy is best.
- r_terminal, last decision only: a failure (off road, missed checkpoint, stuck) -fail_s; a success that ended by slowing
  below 60 kph before s_goal -(s_goal - s_end) / 60 kph, the rest of the way at the limit, so ending early saves nothing.
  A truncated episode (time limit, interrupt) gets none: the learner bootstraps it.

Learner record, one per decision (rewards.jsonl): episode, k, t_wall, t_sim, s, v, phi, phi_next, r_time, r_penalty,
r_shape, r_terminal, r, events [{type, t_sim, penalty_s}], done, terminated, truncated, end (reason on the last decision).
The episode manager (rl_episode.py) writes them on a 10 Hz grid of simulator time and keeps the episode timeline
(epK.timeline.json); the actor's own decision times re-join it:

    rl_reward.py EP.timeline.json DECISIONS.jsonl > rewards.jsonl     DECISIONS: one JSON per line with t_wall (host
                                                                       clock, time.time() at the decision's t0), any
                                                                       other fields (e.g. id) are copied through
"""
import json
import sys

import numpy as np

V60 = 60.0 / 3.6
SUCCESS, FAILURE = ("goal", "slow"), ("off_road", "stuck", "stuck_after_collision", "missed_cp")


def penalty_times(events, cfg):
    """scorer events -> [(t, penalty_s, type)] at the frames the scorer adds each penalty."""
    dt = 1.0 / cfg["frame_hz"]
    out = []
    for e in events:
        p, t0, kind = e["penalty_s"], e["t_start"], e["type"]
        if p <= 0 or e.get("after_finish") or e.get("repeat"):
            continue
        if kind == "speed":                               # (1 + frames // period) x 15: once, then per full period
            unit, per = cfg["speed"]["penalty_s"], cfg["speed"]["repeat_s"]
            out.append((t0, unit, kind))
            out += [(t0 + m * per - dt, unit, kind) for m in range(1, round(p / unit))]
        elif kind == "lane":                              # (frames // period) x 5
            unit, per = cfg["lane"]["penalty_s"], cfg["lane"]["period_s"]
            out += [(t0 + m * per - dt, unit, kind) for m in range(1, round(p / unit) + 1)]
        else:
            out.append((t0, p, kind))
    return sorted(out)


def timeline(labels, summary, cfg, t0_sim, end, t_end, s_goal, clock):
    """Everything the per-decision rewards need, on simulator time (seconds). labels/summary: scorer.score of the
    episode's rows, whose frame t = 0 is simulator time t0_sim; end: the manager's end reason; clock: [(wall, sim)]."""
    ok = np.isfinite(labels.path_s.to_numpy(float))
    return dict(t_end=t_end, end=end, s_goal=s_goal,
                penalties=[(t + t0_sim, p, k) for t, p, k in penalty_times(summary["events"], cfg) if t + t0_sim <= t_end + 1e-6],
                s=[[round(t + t0_sim, 3), round(s, 2), round(v, 3)] for t, s, v in
                   zip(labels.t[ok], labels.path_s[ok], labels.speed_kph[ok] / 3.6)],
                clock=clock)


def rewards(tl, t_dec, v_ref=80 / 3.6, fail_s=100.0):
    """Per-decision records for decision times t_dec (simulator s, ascending) inside the episode."""
    t_dec = np.asarray([t for t in t_dec if t < tl["t_end"]], float)
    if not len(t_dec):
        return []
    S = np.asarray(tl["s"], float)
    s_at = lambda t: np.interp(t, S[:, 0], S[:, 1])
    t_next = np.r_[t_dec[1:], tl["t_end"]]
    s, s_next, end = s_at(t_dec), s_at(t_next), tl["end"]
    reason = end.rstrip("0123456789")                     # missed_cp13 -> missed_cp
    terminated, truncated = reason in SUCCESS + FAILURE, reason not in SUCCESS + FAILURE
    phi, phi_next = s / v_ref, s_next / v_ref
    if terminated:
        phi_next[-1] = tl["s_goal"] / v_ref
    pen = [[] for _ in t_dec]
    for t, p, kind in tl["penalties"]:                    # t in (t_k, t_k+1] -> k; before the first decision -> 0
        pen[min(max(int(np.searchsorted(t_dec, t, side="left")) - 1, 0), len(t_dec) - 1)].append((t, p, kind))
    term = np.zeros(len(t_dec))
    if reason in FAILURE:
        term[-1] = -fail_s
    elif reason == "slow":
        term[-1] = -max(tl["s_goal"] - s_next[-1], 0.0) / V60
    wall = np.asarray(tl["clock"], float)
    t_wall = np.interp(t_dec, wall[:, 1], wall[:, 0])
    out = []
    for k, t in enumerate(t_dec):
        r_time, r_pen, r_shape = -(t_next[k] - t), -sum(p for _, p, _ in pen[k]), phi_next[k] - phi[k]
        last = k == len(t_dec) - 1
        out.append(dict(k=k, t_wall=round(float(t_wall[k]), 3), t_sim=round(float(t), 3), s=round(float(s[k]), 2),
                        v=round(float(np.interp(t, S[:, 0], S[:, 2])), 3), phi=float(phi[k]), phi_next=float(phi_next[k]),
                        r_time=float(r_time), r_penalty=float(r_pen), r_shape=float(r_shape), r_terminal=float(term[k]),
                        r=float(r_time + r_pen + r_shape + term[k]),
                        events=[dict(type=kind, t_sim=round(tp, 3), penalty_s=p) for tp, p, kind in pen[k]],
                        done=last, terminated=last and terminated, truncated=last and truncated, end=end if last else None))
    return out


def main():
    tl = json.load(open(sys.argv[1]))
    dec = [json.loads(l) for l in open(sys.argv[2]) if l.strip()]
    wall = np.asarray(tl["clock"], float)
    inside = [d for d in dec if wall[0, 0] <= d["t_wall"] <= wall[-1, 0]]
    recs = rewards(tl, np.interp([d["t_wall"] for d in inside], wall[:, 0], wall[:, 1]), **tl.get("reward_args", {}))
    for d, r in zip(inside, recs):
        print(json.dumps({**d, **r, "episode": tl.get("episode")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
