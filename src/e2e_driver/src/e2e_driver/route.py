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


class PoseGuard:
    """safety net behind the localization fix (5edc217, which removed the 100-250 m swings and the 1.9e17 m value at
    their source): drop non-finite poses and jumps far beyond any real correction, and run route progress on the
    odometer meanwhile. The margin lets real corrections through at once: when GPS returns at the tunnel exit the
    filter snaps 7-11 m (live, 10/26), up to ~25 m in collection. A new track that stays self-consistent for 1 s (a
    scene reload, a filter restart) is taken over."""

    def __init__(self, hold=1.0, margin=50.0):
        self.hold, self.margin, self.fix, self.cand, self.rejected = hold, margin, None, None, 0

    def ok(self, t, x, y, yaw, v):
        if not np.isfinite([x, y, yaw]).all() or max(abs(x), abs(y)) > 1e6:
            self.rejected += 1; return False
        lim = lambda t1: abs(v) * max(t - t1, 0.0) * 1.5 + self.margin
        if self.fix is None or np.hypot(x - self.fix[1], y - self.fix[2]) <= lim(self.fix[0]):
            self.fix, self.cand = (t, x, y), None; return True
        if self.cand is not None and np.hypot(x - self.cand[2], y - self.cand[3]) <= lim(self.cand[1]):
            self.cand = (self.cand[0], t, x, y)
            if t - self.cand[0] >= self.hold:
                self.fix, self.cand = (t, x, y), None; return True
        else:
            self.cand = (t, t, x, y)
        self.rejected += 1; return False


class TeleportDetector:
    """the Ego moved in one step (a checkpoint return by the operators, a scene reload): the stack is not told, so it
    watches for jumps no driving can make. competition_status heading changing by more than max_yaw between messages
    less than 0.3 s apart (also inside a GPS blackout), or the GPS fix moving more than max_jump + 2 |v| dt (positions
    from NMEA, ~0.2 m steps). An event is reported once; detections within hold s after it are the same event.
    Known gap: a return to a spot with the same heading inside a GPS blackout is not seen (checkpoints lie outside it)."""

    def __init__(self, max_yaw=0.5, max_jump=20.0, hold=2.0):
        self.max_yaw, self.max_jump, self.hold = max_yaw, max_jump, hold
        self.last_status = self.last_fix = self.origin = None
        self.v, self.event_t = 0.0, -1e18

    def _event(self, t):
        fire = t - self.event_t > self.hold
        self.event_t = t
        return fire

    def status(self, t, yaw, v):
        last, self.last_status, self.v = self.last_status, (t, yaw), v
        if last is None or not 0 < t - last[0] < 0.3:
            return False
        return abs(np.arctan2(np.sin(yaw - last[1]), np.cos(yaw - last[1]))) > self.max_yaw and self._event(t)

    def gps(self, t, lat, lon):
        if lat is None or (lat == 0 and lon == 0) or not np.isfinite([lat, lon]).all():
            return False                                            # blackout: keep the last fix
        if self.origin is None:
            self.origin = (lat, lon)
        x = 6378137.0 * np.radians(lon - self.origin[1]) * np.cos(np.radians(self.origin[0]))
        y = 6378137.0 * np.radians(lat - self.origin[0])
        last, self.last_fix = self.last_fix, (t, x, y)
        if last is None:
            return False
        return np.hypot(x - last[1], y - last[2]) > self.max_jump + 2.0 * abs(self.v) * max(t - last[0], 0.0) and self._event(t)


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
