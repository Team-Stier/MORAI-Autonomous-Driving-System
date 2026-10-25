"""Verbatim copies of the training code the ROS side needs (python 3.8 / numpy 1.17, no torch), so the live inputs are
picked and worded exactly as the training samples were. test/test_train_copy.py checks them against the originals:
  Alpamayo-Cosmos train/morai_vla/data.py        N_FRAMES, N_HIST, DT, MAX_FRAME_GAP, MAX_SCAN_GAP, frame_targets,
                                                 nearest, select_distinct
  Alpamayo-Cosmos train/morai_vla/build_index.py nav_text (NAV_RANGE comes from assets/route.json)
Edit only by re-copying.
"""
import numpy as np

NAV_RANGE = 150.0                     # m: farther maneuvers -> "Follow the road"   (build_index.py)
N_FRAMES, N_HIST, N_FUT, DT = 4, 16, 64, 0.1
MAX_FRAME_GAP = 0.08                           # s: a nominal pick further than this from its tau target -> row unusable (drops 1.8 %)
MAX_SCAN_GAP = 0.15


def frame_targets(t0, rng=None, jitter=(0.08, 0.12)):
    """tau targets oldest -> newest: t0 - 0.3 ... t0 on the 0.1 s grid; with rng, each gap drawn from U(jitter)."""
    gaps = rng.uniform(*jitter, N_FRAMES - 1) if rng is not None else np.full(N_FRAMES - 1, DT)
    return t0 - np.r_[np.cumsum(gaps[::-1])[::-1], 0.0]


def nearest(sorted_tau, targets):
    j = np.clip(np.searchsorted(sorted_tau, targets), 1, len(sorted_tau) - 1)
    return np.where(np.abs(sorted_tau[j - 1] - targets) <= np.abs(sorted_tau[j] - targets), j - 1, j)


def select_distinct(tau, targets):
    """nearest frame/scan per target (oldest -> newest), forced strictly older than the next newer pick: dropped camera
    frames (10 % of rows) and 8.4-8.9 Hz scans would otherwise repeat one frame. The time text carries the real offset."""
    k = nearest(tau, targets).copy()                                              # targets (..., 4): rows vectorised
    for n in range(k.shape[-1] - 2, -1, -1):
        k[..., n] = np.minimum(k[..., n], k[..., n + 1] - 1)
    return k


def nav_text(s, M, lap, exempt):
    """next maneuver within NAV_RANGE (distance clamped at 0 inside its junction), plus the speed rule."""
    text = "Follow the road"
    for k in (0, 1):                                           # this lap, then the next lap's first maneuvers
        hit = [(p + k * lap, t) for p, end, t in M if s <= end + k * lap and p + k * lap - s <= NAV_RANGE]
        if hit:
            p, t = hit[0]; text = t.format(d=int(round(max(p - s, 0.0)))); break
    return text + (". No speed limit" if exempt[0] <= s <= exempt[1] else ". Speed limit 60 km/h")
