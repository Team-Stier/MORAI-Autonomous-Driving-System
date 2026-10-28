#!/usr/bin/env python3
"""eval_manager.Stuck on made-up steps (20 Hz): standing, a red light, a yield to a crossing pedestrian, a collision."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import eval_manager as em

E = dict(x=0.0, y=0.0, yaw=0.0, tl=None, color=-1)
RED = dict(E, tl="C1", color=1)


def first(steps, objs=lambda t: []):
    """steps: tau -> (s, ego, hit); the first tau a return is due (to 0.1 s: a step's dt counts from the next step), and why"""
    st = em.Stuck(0.0)
    for i in range(1, 20 * 200):
        t = i / 20
        s, e, hit = steps(t)
        why = st.update(t, s, e, objs(t), hit)
        if why:
            return round(t, 1), why


assert first(lambda t: (0.0, E, False)) == (30.0, "stuck")                          # standing: 30 s
assert first(lambda t: (0.1 * t, E, False)) == (30.0, "stuck")                      # creeping 0.1 m/s: 3 m in 30 s
assert first(lambda t: (0.0, RED if t < 40 else E, False)) == (70.0, "stuck")      # 40 s red, then 30 s green
assert first(lambda t: (0.0, RED, False)) == (90.0, "stuck")                        # red for ever: 90 s at most
ped = lambda t: [(7, 0, 8.0, -5.0 + 1.2 * (t % 10), 0, 0, 0.5, 0.5, 1.7)]          # crossing 8 m ahead at 1.2 m/s
assert first(lambda t: (0.0, E, False), ped) == (90.0, "stuck")                     # yielding: 90 s at most
behind = lambda t: [(7, 0, -8.0, -5.0 + 1.2 * (t % 10), 0, 0, 0.5, 0.5, 1.7)]       # behind the Ego: no yield
assert first(lambda t: (0.0, E, False), behind) == (30.0, "stuck")
parked = lambda t: [(8, 2, 8.0, 0.0, 0, 0, 4.6, 1.9, 1.5)]                          # a still obstacle ahead: no yield
assert first(lambda t: (0.0, E, False), parked) == (30.0, "stuck")
assert first(lambda t: (0.0, E, 1.0 <= t < 1.1)) == (15.0, "stuck_after_collision")
print("stuck ok")
