"""Route progress on the sim clock (Noetic chroot): the node's RouteTracker + Odometer with its GPS-blackout gating, fed
from a collected episode mapped to tau (as offline_replay.py), against the training path_s of the _ego grid. Fixes come
from the bag's /collect_only/localization_ekf (the collection stack's own localization: the same package and the same
tunnel_wall fallback). wall-clock bag_loop.sh runs overstate odometer distance by 1/RTF; this does not.
Usage: python3 route_replay.py EGO_NPZ BAG [BAG ...]   (prints JSON)"""
import json, os, sys

import numpy as np
import rosbag

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from e2e_driver.route import Odometer, Route, RouteTracker      # noqa: E402
from offline_replay import Clock, yaw_of                        # noqa: E402


def main():
    clk, z = Clock(sys.argv[1]), dict(np.load(sys.argv[1]))
    route = Route(os.path.join(HERE, "..", "assets", "route.json"))
    out = {}
    for gate in (False, True):
        tr, odo, gps_ok, rows, k = RouteTracker(route), Odometer(), True, [], 0
        grid = z["tau"]
        for p in sys.argv[2:]:
            for topic, m, t in rosbag.Bag(p).read_messages(topics=["/vehicle/competition_status", "/sensors/gps/fix", "/collect_only/localization_ekf"]):
                ts = float(clk.tau_at(m.header.stamp.to_nsec()))
                while k < len(grid) and grid[k] < ts:                  # sample s on the 10 Hz grid as the node would
                    s = tr.s_at(odo.at(grid[k]))
                    if s is not None: rows.append((k, s))
                    k += 1
                if topic == "/vehicle/competition_status":
                    odo.add(ts, m.signed_velocity_mps)
                elif topic == "/sensors/gps/fix":
                    gps_ok = not (m.status.status < 0 or (m.latitude == 0 and m.longitude == 0))
                elif not gate or gps_ok:
                    q = m.pose.pose
                    tr.update(ts, q.position.x, q.position.y, yaw_of(q.orientation), odo.d)
        i = np.array([r[0] for r in rows]); s = np.array([r[1] for r in rows])
        e = s - z["path_s"][i].astype(np.float64)
        e = (e + route.lap / 2) % route.lap - route.lap / 2             # a lap apart is the same place (loop start/end)
        blk, sec = z["gps_blackout"][i].astype(bool), z["section"][i]
        texts = [(route.nav_text(a), route.nav_text(float(b))) for a, b in zip(s, z["path_s"][i])]
        nav = np.array([a == b for a, b in texts])
        head = lambda t: "".join(c for c in t if not c.isdigit())        # maneuver and speed rule without the distance
        near = np.array([a != b and head(a) == head(b) for a, b in texts])
        other = [(a, b) for a, b in texts if a != b and head(a) != head(b)]
        st = lambda x: dict(n=int(len(x)), median=round(float(np.median(np.abs(x))), 2), max=round(float(np.abs(x).max()), 2)) if len(x) else None
        out["gps_gated" if gate else "every_fix"] = dict(all=st(e), gps_blackout=st(e[blk]), section15=st(e[sec == 15]),
                                                          nav_equal=round(float(nav.mean()), 4), nav_distance_only=round(float(near.mean()), 4),
                                                          nav_other=round(len(other) / len(texts), 4), nav_other_examples=other[:3],
                                                          nav_equal_blackout=round(float(nav[blk].mean()), 4) if blk.any() else None)
        if gate and os.environ.get("DEBUG_ROWS"):
            bad = np.flatnonzero(np.abs(e) > 20)
            print("big errors:", len(bad), [(round(float(z["tau"][i[b]]), 1), round(float(s[b]), 1), round(float(z["path_s"][i[b]]), 1)) for b in bad[::max(1, len(bad) // 12)]])
    print(json.dumps(out))


if __name__ == "__main__":
    main()
