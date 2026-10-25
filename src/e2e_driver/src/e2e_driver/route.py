"""Route progress s and nav text at inference, as the training rows got them.

Training: path_s = Alpamayo-Cosmos tools/scorer KMap.progress on the GT base_link pose (Ego status, rear axle), linearly
interpolated onto the tau grid; nav_text = build_index.nav_text(path_s). Here the pose is /localization/ego (the stack's
GPS/IMU EKF, base_link in the same map frame: median 0.09 m from GT in run1024b-w1-001) and RouteTracker.update is
KMap.progress's loop body run one pose at a time. While localization is missing (GPS blackout: the EKF does not publish,
and in the tunnel section the LiDAR-wall core has no point cloud), s advances with the competition-status odometer.
"""
import json

import numpy as np

from e2e_driver.train_copy import nav_text


class Route:
    def __init__(self, asset):
        a = json.load(open(asset))
        self.path, self.s, self.heading = np.array(a["path"]), np.array(a["s"]), np.array(a["heading"])
        self.M = [(p, e, t) for p, e, t in a["maneuvers"]]
        self.lap, self.exempt, self.nav_range = a["lap_m"], tuple(a["exempt_s"]), a["nav_range_m"]

    def nav_text(self, s):
        return nav_text(s, self.M, self.lap, self.exempt)

    def pursuit_kappa(self, s, x, y, yaw, lookahead):
        """test driver only (fake backend load runs): pure-pursuit curvature toward the route point `lookahead` m past s."""
        i = min(int(np.searchsorted(self.s, (s + lookahead) % self.lap)), len(self.s) - 1)
        dx, dy = self.path[i, 0] - x, self.path[i, 1] - y
        return 2.0 * np.sin(np.arctan2(dy, dx) - yaw) / max(np.hypot(dx, dy), 1.0)


class RouteTracker:
    """KMap.progress online. The vertex index i is searched in [i - 40, i + 400) while the window has a vertex within
    5 m; otherwise globally, among vertices heading the same way, lowest s among near ties. After a localization gap
    longer than `reseed_after` the window starts from the dead-reckoned s instead of the stale vertex."""

    def __init__(self, route, reseed_after=1.0):
        self.r, self.reseed_after = route, reseed_after
        self.i = None                      # vertex index of the last fix
        self.fix_t = self.fix_odo = None   # time and odometer reading of the last fix

    def update(self, t, x, y, yaw, odo):
        p, h, P = np.array([x, y]), yaw, self.r.path
        i = self.i
        if i is not None and self.fix_t is not None and t - self.fix_t > self.reseed_after:
            i = int(np.clip(np.searchsorted(self.r.s, self.s_at(odo)), 0, len(P) - 1))
        if i is not None:
            lo = max(i - 40, 0)
            win = np.hypot(*(P[lo:i + 400] - p).T)
            i = lo + int(win.argmin()) if win.min() < 5.0 else None
        if i is None:
            d = np.hypot(*(P - p).T)
            if np.isfinite(h):
                d[:-1][np.cos(self.r.heading - h) < 0] = np.inf
            i = int(np.flatnonzero(d <= d.min() + 0.5)[0])
        self.i, self.fix_t, self.fix_odo = i, t, odo
        return float(self.r.s[i])

    def s_at(self, odo):
        """s now: the last fix's vertex s plus the distance driven since (competition-status odometer)."""
        if self.i is None:
            return None
        return float(self.r.s[self.i]) + (odo - self.fix_odo)


class Odometer:
    """distance driven (m): trapezoid over competition_status signed speed samples."""

    def __init__(self):
        self.t = self.v = None
        self.d = 0.0

    def add(self, t, v):
        if self.t is not None and t > self.t:
            self.d += 0.5 * (v + self.v) * (t - self.t)
        if self.t is None or t > self.t:
            self.t, self.v = t, v
        return self.d

    def at(self, t):
        return self.d if self.t is None else self.d + self.v * max(t - self.t, 0.0)
