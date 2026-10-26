"""Live simulator clock (e2e_driver.sim_clock, causal, GPS + competition_status) against the tau the training rows were
built on (step-0 _ego table: coc_labeler load_episode, centred 5 s windows on GT pose), over collected bags.
Run in the chroot: python3 sim_clock_check.py EGO_NPZ BAG [BAG ...]
Prints the tau difference over the episode (after the first rho update, both counted from there) and rho vs the
training's local rate."""
import os, sys

import numpy as np
import rosbag

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from e2e_driver.sim_clock import SimClock  # noqa: E402


def main():
    z = np.load(sys.argv[1]); ego_ns, ego_tau = z["bag_ns"].astype(np.float64), z["tau"].astype(np.float64)
    msgs = []
    for b in sys.argv[2:]:
        for topic, m, t in rosbag.Bag(b).read_messages(topics=["/vehicle/competition_status", "/sensors/gps/fix"]):
            msgs.append((t.to_sec(), topic, m))
    msgs.sort(key=lambda x: x[0])
    c = SimClock(chord=float(os.environ.get("CHORD", "2.0"))); rows = []
    for t, topic, m in msgs:
        if topic == "/sensors/gps/fix":
            ok = m.status.status >= 0
            c.add_gps(t, m.latitude if ok else None, m.longitude if ok else None)
        else:
            c.add_status(t, m.signed_velocity_mps)
            rows.append((t, c.advance(t), c.rho, c.estimated))
    r = np.array(rows)
    first = int(np.argmax(r[:, 3] > 0))
    r = r[first:]
    t, tau, rho = r[:, 0], r[:, 1], r[:, 2]
    ref = np.interp(t * 1e9, ego_ns, ego_tau)
    d = (tau - tau[0]) - (ref - ref[0])
    k = 50                                                     # local rates over ~5 s of status messages
    loc = (ref[k:] - ref[:-k]) / (t[k:] - t[:-k])
    print("%s: %.0f s from the first rho update (%.1f s in); tau diff end %+.2f s (%.2f %%), max |diff| %.2f s; "
          "rho median %.3f vs training rate median %.3f, |rho - rate| median %.3f p95 %.3f"
          % (os.path.basename(sys.argv[1])[:-4], t[-1] - t[0], t[0] - msgs[0][0], d[-1], 100 * d[-1] / (ref[-1] - ref[0]),
             np.abs(d).max(), np.median(rho), np.median(loc), np.median(np.abs(rho[k:] - loc)), np.percentile(np.abs(rho[k:] - loc), 95)))


if __name__ == "__main__":
    main()
