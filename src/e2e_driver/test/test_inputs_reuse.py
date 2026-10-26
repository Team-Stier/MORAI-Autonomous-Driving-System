"""Inputs(reuse=True): at 10 Hz requests, each request brings one new camera frame per camera (the history slots reuse
the earlier current frames, within REUSE_TOL of their targets); reuse=False re-picks the nearest buffered frame per
slot, as training did. Cameras at 30 Hz 40 ms late (every step 4 new frames instead of 8) and at an irregular ~11.5 Hz
(the bag's 10 Hz copies, frame age ~40 ms at a step, as the fake host showed).
    python3 test_inputs_reuse.py           (numpy, opencv)
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from e2e_driver.inputs import CAMERAS, REUSE_TOL, Inputs  # noqa: E402


def frame(i):
    im = np.full((32, 48, 3), (i * 37) % 256, np.uint8); im[0, 0] = i % 251
    return cv2.imencode(".jpg", im)[1].tobytes()


def run(times, reuse, steps=60, lag=0.04):
    inp, prev, new, errs, i = Inputs(horizon=3.0, reuse=reuse), None, [], [], 0
    for n in range(steps):
        t0 = 1.0 + 0.1 * n
        while i < len(times) and times[i] <= t0 - lag:             # what has arrived by t0
            for c in CAMERAS:
                inp.add_camera(c, float(times[i]), frame(i))
            i += 1
        for t in np.arange(t0 - 2.0, t0 + 1e-9, 0.02):
            inp.add_status(float(t), 5.0, 0.0)
        req, _ = inp.request(t0, "", lidar=False)
        if req is None:
            continue
        cur = [j for cam in req["camera_jpegs"] for j in cam]
        if prev is not None:
            new.append(sum(j not in prev for j in cur))
        prev, errs = set(cur), errs + [abs(e) for row in req["pick"]["err_ms"] for e in row[:-1]]
    return new[5:], errs


rng = np.random.default_rng(0)
for name, times, lag in [("30 Hz", np.arange(0, 8, 1 / 30), 0.04),
                         ("11.5 Hz jitter", np.cumsum(rng.uniform(0.06, 0.115, 120)), 0.005)]:
    on, errs = run(times, True, lag=lag)
    off, _ = run(times, False, lag=lag)
    print("%-15s reuse: new/step counts %s mean %.2f, slot err p50 %.0f max %.0f ms | re-pick: counts %s mean %.2f" % (
        name, np.bincount(on).tolist(), np.mean(on), np.median(errs), max(errs), np.bincount(off).tolist(), np.mean(off)))
    if name == "30 Hz":
        assert set(on) == {len(CAMERAS)} and max(errs) <= 1e3 * REUSE_TOL, (on, max(errs))   # one new frame per camera
    assert np.mean(on) <= np.mean(off), (on, off)
print("ok")
