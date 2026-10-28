#!/usr/bin/env python3
"""[SEGMENTS=dyn,round,hwy,tunnel] [FULL_SHARE=0.2] segments.py N | --args SEGMENT: the DAgger actor split (user
10/28): the segment presets, CPs of the 10/7 checkpoint table with room before (the entry decision) and after (the exit
hand-over). Within a segment a failure returns to the last CP (--returns) and the segment's end starts the next episode
with a new scene. FULL_SHARE of the actors drive the whole lap (forgetting, a lap score per LoRA version); the rest are
split by the segments' weights over the chosen ones, largest remainder, every chosen segment at least one actor.
N: prints the split and one line per actor (segment, its rl_episode arguments). --args: one segment's arguments
(actor.sh SEGMENT=)."""
import argparse
import os

# segment -> (rl_episode range arguments, weight). dyn / tunnel: shadow-dyn's (38b340d: scenes whose movers make the Ego
# brake, the run past the end CP not cut by the "slow" end); round: shadow-round's runs, same end; hwy: CP9 for the
# on-ramp merge at CP10 (1118.7 m), the official slow end after CP12 kept.
SEGMENTS = dict(
    dyn=("--start-cp 4 --end-cp 5 --past-m 90 --no-slow-end --scene-preset brake", 30),
    round=("--start-cp 7 --end-cp 9 --past-m 60 --no-slow-end", 25),
    hwy=("--start-cp 9 --end-cp 12 --past-m 100", 25),
    tunnel=("--start-cp 12 --end-cp 13 --past-m 125 --no-slow-end --scene-preset brake", 20),
)
FULL = "--start-cp 0 --end-cp 14 --past-m 100"
COMMON = "--returns"


def split(n, chosen, full_share):
    """-> {name: actors}: full_share of n for "full", the rest by weight, largest remainder, chosen segments >= 1."""
    w = sum(SEGMENTS[s][1] for s in chosen)
    quota = dict(full=n * full_share, **{s: n * (1 - full_share) * SEGMENTS[s][1] / w for s in chosen})
    out = {k: int(q) for k, q in quota.items()}
    for s in chosen:
        out[s] = max(out[s], 1)
    by_rem = sorted(quota, key=lambda k: quota[k] - int(quota[k]), reverse=True)
    i = 0
    while sum(out.values()) < n:
        out[by_rem[i % len(by_rem)]] += 1
        i += 1
    while sum(out.values()) > n:                         # the minimum of one went over: take from the largest share
        k = max((k for k in out if out[k] > (1 if k in chosen else 0)), key=lambda k: out[k])
        out[k] -= 1
    return out


def args_of(name):
    return f"{FULL if name == 'full' else SEGMENTS[name][0]} {COMMON}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("n", nargs="?", type=int)
    ap.add_argument("--segments", default=os.environ.get("SEGMENTS", "dyn,round,hwy,tunnel"))
    ap.add_argument("--full-share", type=float, default=float(os.environ.get("FULL_SHARE", 0.2)))
    ap.add_argument("--args", help="print one segment's rl_episode arguments")
    a = ap.parse_args()
    if a.args:
        print(args_of(a.args))
        return
    chosen = [s for s in a.segments.split(",") if s]
    assert chosen and all(s in SEGMENTS for s in chosen), f"segments: {', '.join(SEGMENTS)}"
    assert a.n >= len(chosen), f"{a.n} actors for {len(chosen)} segments"
    out = split(a.n, chosen, a.full_share)
    print("split: " + ", ".join(f"{k} {v}" for k, v in out.items()) + f" (n {a.n}, full share {a.full_share})")
    k = 0
    for name, m in out.items():
        for _ in range(m):
            print(f"actor {k}: SEGMENT={name} {args_of(name)}")
            k += 1


if __name__ == "__main__":
    assert split(11, list(SEGMENTS), 0.2) == dict(full=2, dyn=3, round=2, hwy=2, tunnel=2)    # the user's 11-actor example
    assert split(4, ["round", "hwy"], 0.2) == dict(full=1, round=2, hwy=1)    # remainders: full .8, round .6, hwy .6
    assert split(3, list(SEGMENTS)[:3], 0.2) == dict(full=0, dyn=1, round=1, hwy=1)
    main()
