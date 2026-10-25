"""GridYawRate checks (numpy only): constant rate is exact for hold and trend; the trend follows a turn onset sooner."""
import os, sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from vlp16_bev.scan import GridYawRate  # noqa: E402


def test_grid_yaw_rate():
    for tw in (0.0, 0.3, 0.6):
        g = GridYawRate(trend_window=tw)
        for t in np.arange(0, 3.0, 0.05): g.add(t, 0.2 * t)
        assert np.allclose(g.at([2.0, 2.6, 2.95]), 0.2), tw
    hold, trend = GridYawRate(), GridYawRate(trend_window=0.3)
    for t in np.arange(0, 3.0, 0.05):
        y = 0.0 if t < 2.6 else 0.4 * (t - 2.6)
        hold.add(t, y); trend.add(t, y)
    assert hold.at([2.95])[0] < trend.at([2.95])[0]


if __name__ == "__main__":
    test_grid_yaw_rate(); print("scan ok")
