"""Simulator clock tau estimated live, the way the training rows got it.

MORAI sends no simulation time: every stamp it gives (competition_status source time, camera and VLP-16 packet times)
is the simulator host's system clock, and the simulator often runs slower than real time (RTF 0.6-0.95). Training
rebuilt tau per episode in coc_labeler data.load_episode: rho = sum |d pos| / sum (v dt) over 5 s windows (clipped to
0.5-1.05, gaps interpolated, 0.8 if unknown) and tau = integral of rho dt. Live, the same ratio is taken over the
trailing 5 s from the GPS fix (position) and competition_status (speed, the simulator's own m/s), both on the wall
clock of their receipt:
  - path: GPS chords >= 2 s apart (GPS noise and ~0.1-0.2 m NMEA quantisation inflate a 30 Hz path length; on 3
    collected episodes 2 s chords track the training tau to +4.2, +0.9 and -0.5 % over 3 min, 1 s chords +4.3, +1.3, -3.1);
  - rho is updated only while moving (sum v dt > 2 m, as in training) with GPS valid over the whole window, and the last
    value is held otherwise (standstill, GPS blackout);
  - tau stops while the status stream does (a simulator stall: no simulation time passes).
"""
import math
from collections import deque

R_EARTH = 6378137.0


class SimClock:
    def __init__(self, rho0=0.8, window=5.0, chord=2.0, min_den=2.0, lo=0.3, hi=1.05, stall=0.3):
        self.rho, self.window, self.chord, self.min_den, self.lo, self.hi, self.stall = rho0, window, chord, min_den, lo, hi, stall
        self.v, self.gps = deque(), deque()          # (t, v) and (t, x, y) or (t, None, None) for a blackout fix
        self.origin = None
        self.t = self.tau = self.last_status = None
        self.estimated = 0                           # number of rho updates (0: still the prior)

    def reset_window(self):
        """a teleport: the GPS path across it is not driving; rho is kept and estimated afresh after it."""
        self.v.clear(); self.gps.clear()

    def add_status(self, t, v):
        self.v.append((t, abs(v))); self.last_status = t
        while self.v and self.v[0][0] < t - self.window - 1.0:
            self.v.popleft()

    def add_gps(self, t, lat, lon):
        if lat is None or not (math.isfinite(lat) and math.isfinite(lon)) or (lat == 0 and lon == 0):
            self.gps.append((t, None, None))
        else:
            if self.origin is None:
                self.origin = (lat, lon)
            la0, lo0 = self.origin
            self.gps.append((t, R_EARTH * math.radians(lon - lo0) * math.cos(math.radians(la0)), R_EARTH * math.radians(lat - la0)))
        while self.gps and self.gps[0][0] < t - self.window:
            self.gps.popleft()
        self._estimate()

    def _estimate(self):
        g = list(self.gps)
        if len(g) < 2 or any(p[1] is None for p in g) or g[-1][0] - g[0][0] < 0.8 * self.window:
            return
        pts = [g[0]]
        for p in g[1:]:
            if p[0] - pts[-1][0] >= self.chord:
                pts.append(p)
        ta, tb = pts[0][0], pts[-1][0]
        num = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(pts, pts[1:]))
        vs = [s for s in self.v if ta <= s[0] <= tb]
        den = sum(0.5 * (a[1] + b[1]) * (b[0] - a[0]) for a, b in zip(vs, vs[1:]))
        if den > self.min_den:
            self.rho = min(max(num / den, self.lo), self.hi); self.estimated += 1

    def advance(self, t):
        """wall time t -> tau; tau starts at the first call's wall time and runs at rho while the status stream is live."""
        if self.t is None:
            self.t, self.tau = t, t
        dt, self.t = t - self.t, t
        if dt > 0 and self.last_status is not None and t - self.last_status < self.stall:
            self.tau += self.rho * dt
        return self.tau
