"""SimClock checks (no ROS): RTF 0.6 on a curved path with quantised GPS, blackout hold, stall freeze."""
import math, os, sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from e2e_driver.sim_clock import R_EARTH, SimClock  # noqa: E402


def test_sim_clock():
    rtf, v, r = 0.6, 7.0, 30.0                       # 7 m/s on a 30 m circle, simulator at 0.6x real time
    c = SimClock()
    lat0, lon0 = 37.24, 126.77
    t = 1000.0
    for k in range(2000):                             # 20 s of wall time, status 20 Hz, GPS 30 Hz (wall)
        t = 1000.0 + k * 0.01
        if k % 5 == 0:
            c.add_status(t, v)
        if k % 3 == 0:
            s = v * rtf * (t - 1000.0)                # simulation distance after t - 1000 wall seconds
            x, y = r * math.sin(s / r), r * (1 - math.cos(s / r))
            lat = round(lat0 + math.degrees(y / R_EARTH), 6)                                  # NMEA-like quantisation
            lon = round(lon0 + math.degrees(x / (R_EARTH * math.cos(math.radians(lat0)))), 6)
            c.add_gps(t, lat, lon)
        c.advance(t)
    assert abs(c.rho - rtf) < 0.02, c.rho
    tau0 = c.advance(t + 0.01)
    for k in range(100):                              # 1 s GPS blackout while still driving: rho held
        c.add_status(t + 0.02 + 0.05 * k, v); c.add_gps(t + 0.02 + 0.05 * k, 0.0, 0.0)
    held = c.rho
    assert abs(held - rtf) < 0.02
    c.advance(t + 5.0)
    frozen = c.advance(t + 8.0)                       # no status for > 3 s: a stall, tau stands still
    assert frozen == c.advance(t + 9.0) and frozen > tau0


if __name__ == "__main__":
    test_sim_clock(); print("ok")
