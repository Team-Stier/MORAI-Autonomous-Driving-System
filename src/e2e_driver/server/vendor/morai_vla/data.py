"""MORAI step-0 dataset for Alpamayo-1.5-style training (MODEL_PLAN.md 2.1-2.5). Shared by teacher and student.

Root layout (tools/step0/README.md): <root>/<ep>/{cams.bin, cams_index.npz, lidar.bin, lidar_index.npz}, <root>/_ego/<ep>.npz,
<root>/_index/index.parquet (morai_vla.build_index). lidar.bin comes from tools/step0/step0_worker.py `lidar` mode
(morai_vla.lidar_bev: per-scan BEV, motion from GT speed + yaw rate; pose = that odometry at the scan's last packet).
"""
import io, logging, zlib
from functools import lru_cache
from pathlib import Path

import numpy as np, pandas as pd, torch
from hydra.utils import instantiate
from PIL import Image

from morai_vla.lidar_bev import pack_bev

log = logging.getLogger(__name__)

N_FRAMES, N_HIST, N_FUT, DT = 4, 16, 64, 0.1
MAX_FRAME_GAP = 0.08                           # s: a nominal pick further than this from its tau target -> row unusable (drops 1.8 %)
MAX_SCAN_GAP = 0.15


def unit_hash(*key):
    return zlib.crc32("/".join(map(str, key)).encode()) / 2**32


def lidar_dropped(episode, i, p, seed=0):
    """The LiDAR modality-dropout mask: a pure function of the sample key, so the student (and soft-target generation)
    drops exactly the same samples as the teacher by calling this with the same (p, seed)."""
    return p > 0 and unit_hash("lidar", seed, episode, int(i)) < p


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


def ego_trajectory(v, yaw, i):
    """history (16 pts, t0-1.5 ... t0) and future (64 pts, t0+0.1 ... t0+6.4) in the ego frame at grid row i, as Alpamayo's
    ego_{history,future}_{xyz,rot} (z = 0, yaw-only rotations); history[-1] is the origin.
    Positions are integrated from GT speed and yaw on the tau grid, not taken from the grid's x, y: those come from Ego status
    messages placed by their arrival time, and that jitter makes the speed from position steps swing +-30 % between
    neighbouring rows (normalised accel std 9.9 instead of ~1). Speed and yaw are smooth, so their jitter is negligible."""
    j = np.arange(i - N_HIST + 1, i + N_FUT + 1)
    h = yaw[j] - yaw[i]                                                           # headings relative to t0 (yaw is unwrapped)
    step = 0.5 * (v[j][1:] + v[j][:-1]) * DT; hm = 0.5 * (h[1:] + h[:-1])
    x, y = np.r_[0, np.cumsum(step * np.cos(hm))], np.r_[0, np.cumsum(step * np.sin(hm))]
    x, y = x - x[N_HIST - 1], y - y[N_HIST - 1]
    xyz = np.stack([x, y, np.zeros(len(j))], -1).astype(np.float32)
    cy, sy = np.cos(h), np.sin(h)
    rot = np.zeros((len(j), 3, 3), np.float32); rot[:, 0, 0], rot[:, 0, 1], rot[:, 1, 0], rot[:, 1, 1], rot[:, 2, 2] = cy, -sy, sy, cy, 1
    t = lambda a: torch.from_numpy(a)[None]                                       # (1, T, ...) = one trajectory group
    return t(xyz[:N_HIST]), t(rot[:N_HIST]), t(xyz[N_HIST:]), t(rot[N_HIST:])


class Episode:
    def __init__(self, d, ego):
        ci = np.load(d / "cams_index.npz"); li = np.load(d / "lidar_index.npz"); z = np.load(ego)
        self.dir = d
        self.cams = []                                                            # per camera: tau-sorted (tau, offset, length)
        for c in range(4):
            m = ci["cam"] == c; o = np.argsort(ci["tau"][m], kind="stable")
            self.cams.append((ci["tau"][m][o].astype(np.float64), ci["offset"][m][o], ci["length"][m][o]))
        self.scan_tau, self.scan_off, self.scan_len, self.scan_pose = li["tau"].astype(np.float64), li["offset"], li["length"], li["pose"]
        self.v, self.yaw, self.tau = z["v"].astype(np.float64), z["yaw"], z["tau"]
        for f, end in (("cams.bin", (ci["offset"] + ci["length"]).max()), ("lidar.bin", (li["offset"] + li["length"]).max())):
            if (d / f).stat().st_size < end: raise ValueError(f"{d / f} shorter than its index (copy in progress?)")

    def usable(self, rows):
        """rows whose nominal frames and scans all exist close to their tau targets."""
        t0 = self.tau[rows][:, None] - DT * np.arange(N_FRAMES)[None, ::-1]
        ok = np.ones(len(rows), bool)
        for tau, _, _ in self.cams:
            if len(tau) < 8: return np.zeros(len(rows), bool)
            k = select_distinct(tau, t0)
            ok &= (k[:, 0] >= 0) & (np.abs(tau[k] - t0) <= MAX_FRAME_GAP).all(1)
        if len(self.scan_tau) < 8: return np.zeros(len(rows), bool)
        k = nearest(self.scan_tau, t0[:, -1])
        return ok & (np.abs(self.scan_tau[k] - t0[:, -1]) <= MAX_SCAN_GAP) & (k >= N_FRAMES - 1)

    def jpeg(self, c, k):
        _, off, ln = self.cams[c]
        with open(self.dir / "cams.bin", "rb") as f:
            f.seek(int(off[k])); b = f.read(int(ln[k]))
        return torch.from_numpy(np.array(Image.open(io.BytesIO(b)).convert("RGB"))).permute(2, 0, 1)

    def scan(self, k):
        with open(self.dir / "lidar.bin", "rb") as f:
            f.seek(int(self.scan_off[k])); b = f.read(int(self.scan_len[k]))
        return np.frombuffer(zlib.decompress(b), np.uint8).reshape(2, 320, 160)


class MoraiDataset(torch.utils.data.Dataset):
    """One sample = one tau-grid row: 4 cameras x 4 frames, 2 LiDAR BEVs, ego history/future, nav text, [CoC].

    The recipe's two modes (alpamayo1_sft README "CoC reasoning enabled"; AR1 5.1-5.2), never a mix:
      coc=False: every usable row of the split, trajectory only (Stage 1-T, Stage 2, step 2).
      coc=True:  only the CoC keyframes of `coc_parquet` (`use == True`), t0 = the keyframe's tau0 (Stage 1-C).
    frame_stride: keep every n-th grid row (coc=False). cell_keep: {cell: keep prob} to downsample overfull cells
    (fixed per row via a hash; coc=False). max_samples: deterministic subset (overfit tests).
    rule_ok: parquet (episode, i, rule_ok) from the rule check of the teacher's soft-target trajectories; adds `vkd_ok`
    (False for rows it does not list), which gates the student's velocity-field distillation.
    """

    def __init__(self, root, split="train", model_config=None, vla_preprocess_args=None, coc=False, coc_parquet=None,
                 frame_stride=1, cell_keep=None, max_samples=None, lidar_dropout=0.0, jitter=(0.08, 0.12), augment=True,
                 seed=0, index="_index/index.parquet", rule_ok=None):
        self.root = Path(root); self.seed, self.augment, self.jitter, self.lidar_dropout = seed, augment, tuple(jitter), lidar_dropout
        df = pd.read_parquet(self.root / index, columns=["episode", "i", "cell", "split", "nav_text"])
        df = df[df.split.isin([split] if isinstance(split, str) else list(split))]
        if coc:
            c = pd.read_parquet(coc_parquet, columns=["episode", "tau0", "coc", "use"])
            c = c[c.use].assign(i=lambda d: np.rint(d.tau0 / DT).astype(int))
            assert (np.abs(c.i * DT - c.tau0) < 1e-6).all(), "CoC keyframes off the 0.1 s tau grid"
            df = df.merge(c[["episode", "i", "coc"]].drop_duplicates(["episode", "i"]), on=["episode", "i"])
            log.info("CoC %s: %d of %d use==True keyframes are usable rows of %s", coc_parquet, len(df), len(c), split)
        else:
            keep = df.i.to_numpy() % frame_stride == 0
            for cell, p in (cell_keep or {}).items():
                m = (df.cell == cell).to_numpy()
                keep[m] &= np.array([unit_hash("cell", seed, e, i) < p for e, i in zip(df.episode[m], df.i[m])])
            df = df[keep].assign(coc=None)
        if max_samples:                                                           # only open the episodes of a random superset
            df = df.sample(min(len(df), 4 * max_samples), random_state=seed)
        parts, skipped = [], 0
        for ep, g in df.groupby("episode", observed=True):                        # drop episodes/rows without (complete) step-0 data
            try:
                parts.append(g[self._episode(ep).usable(g.i.to_numpy())])
            except (FileNotFoundError, ValueError):
                skipped += 1
        if skipped: log.warning("MoraiDataset: skipped %d episodes with missing or incomplete step-0 files", skipped)
        df = pd.concat(parts) if parts else df.iloc[:0]
        if max_samples:
            df = df.iloc[np.random.default_rng(seed).choice(len(df), min(len(df), max_samples), replace=False)]
        self.rows = df.reset_index(drop=True)
        if rule_ok:
            r = pd.read_parquet(rule_ok, columns=["episode", "i", "rule_ok"]).astype({"episode": str, "i": int})
            self.rows = self.rows.astype({"episode": str}).merge(r, on=["episode", "i"], how="left")
            self.rows["rule_ok"] = self.rows.rule_ok.fillna(False).astype(bool)
            log.info("rule_ok %s: %d of %d rows pass", rule_ok, self.rows.rule_ok.sum(), len(self.rows))
        log.info("MoraiDataset %s coc=%s: %d samples from %d episodes", split, coc, len(self.rows), self.rows.episode.nunique())
        self.preprocess = instantiate(vla_preprocess_args, model_config=model_config) if vla_preprocess_args is not None else None

    @lru_cache(maxsize=64)
    def _episode(self, ep):
        return Episode(self.root / ep, self.root / "_ego" / f"{ep}.npz")

    def __len__(self):
        return len(self.rows)

    def sample(self, idx, rng=None):
        """raw sample (no tokenization). rng=None: nominal 0.1 s frame spacing."""
        r = self.rows.iloc[idx]; ep = self._episode(r.episode); i = int(r.i); t0 = float(ep.tau[i])
        tgt = frame_targets(t0, rng, self.jitter)
        frames, times = [], []
        for c, (tau, _, _) in enumerate(ep.cams):
            k = np.maximum(select_distinct(tau, tgt), 0)                          # jittered targets near a bag start
            frames.append(torch.stack([ep.jpeg(c, j) for j in k])); times.append(tau[k] - t0)
        out = dict(camera_frames=frames, camera_times=np.stack(times), key=f"{r.episode}/{i}", nav_text=str(r.nav_text),
                   lidar_dropped=lidar_dropped(r.episode, i, self.lidar_dropout, self.seed))
        if not out["lidar_dropped"]:
            k = np.maximum(select_distinct(ep.scan_tau, tgt), 0)                     # [k0 oldest .. k3 newest] -> BEV(k1<-k0), BEV(k3<-k2)
            sc = [ep.scan(j) for j in k]
            out["bev_frames"] = torch.from_numpy(np.stack([pack_bev(sc[n + 1], sc[n], ep.scan_pose[k[n + 1]], ep.scan_pose[k[n]]) for n in (0, 2)]))
            out["bev_times"] = ep.scan_tau[k[[1, 3]]] - t0
        out["cot"] = r.coc if isinstance(r.coc, str) else None
        if "rule_ok" in r: out["vkd_ok"] = bool(r.rule_ok)
        (out["ego_history_xyz"], out["ego_history_rot"], out["ego_future_xyz"], out["ego_future_rot"]) = ego_trajectory(ep.v, ep.yaw, i)
        return out

    def __getitem__(self, idx):
        rng = np.random.default_rng() if self.augment else None                   # fresh entropy per call/worker
        s = self.sample(idx, rng)
        if self.preprocess is not None:
            s["tokenized_data"] = self.preprocess(data=s)
            for k in ("camera_frames", "bev_frames", "camera_times", "bev_times"): s.pop(k, None)   # tokenized already; keep batches light
        return s
