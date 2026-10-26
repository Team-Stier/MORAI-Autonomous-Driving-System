"""(a, kappa) -> terminal control: the fixed tables of control.py vs a small MLP, on step-0 rows (label_control_check.py's
data path). Mac CPU: uv run --with torch,numpy,pandas,pyarrow,scipy,einops,colorlog,hydra-core,pillow,pyyaml.

    python control_mlp.py data EGO_DIR LABELS_DIR CONFIG.json ALPAMAYO_PKG_DIR OUT.npz [rows]
    python control_mlp.py train OUT.npz WEIGHTS_PREFIX

data: rows spread over every episode, more of hard braking (GT a < -2 m/s^2), standstill (v < 0.3) and sharp curves
(|yaw rate / v| > 0.05): inputs = the label (a, kappa) of steps 0-9 (t0 ... t0 + 1 s), v0, the applied accel, brake,
steering (competition_status, left + deg) at t0 - 0.1 and t0; target = the applied values at t0 + 0.1 s (the command at
t0 is applied 0.1 s later, control.py's lookahead); map = control.py's command for the same row. The label tables
record u = accel - brake, split as accel = max(u, 0), brake = max(-u, 0) (one pedal at a time).
train: episode-held-out validation (20 % of episodes); MLP 2 x 128 (SiLU), sigmoid pedals, tanh x 40 deg steering;
trained on all train rows and on half of them, with and without the applied-history inputs; errors by speed and
curvature bin against the map on the same validation rows. Writes WEIGHTS_PREFIX{,_nohist}.npz for control.py mode mlp.
"""
import glob
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SPEED_BINS = [("stop", 0.0, 0.3), ("low", 0.3, 4.5), ("mid", 4.5, 7.5), ("cruise", 7.5, 99.0)]
CURV_BINS = [("straight", 0.0, 0.01), ("curve", 0.01, 0.05), ("sharp", 0.05, 9.0)]
N_STEPS = 10


def data(ego_dir, labels_dir, cfg_path, pkg, out, n_rows=30000):
    import pandas as pd
    import torch
    sys.path[:0] = [pkg, str(HERE.parent / "server" / "vendor"), str(HERE.parent / "src")]
    from alpamayo_r1.action_space import UnicycleAccelCurvatureActionSpace
    from e2e_driver.control import Controller, Plan
    from morai_vla.data import N_FUT, N_HIST, ego_trajectory
    cfg = {k: v for k, v in json.load(open(cfg_path))["action_space_cfg"].items() if k != "_target_"}
    space, ctl, rng = UnicycleAccelCurvatureActionSpace(**cfg), Controller(), np.random.default_rng(0)
    files = sorted(glob.glob(os.path.join(ego_dir, "*.npz")))
    per_ep = int(math.ceil(n_rows / len(files) * 1.15))
    cols = {k: [] for k in ("ep", "x", "y", "map", "v", "k1")}
    for n, f in enumerate(files):
        ep = Path(f).stem
        lf = glob.glob(os.path.join(labels_dir, "*", ep + ".labels.parquet"))
        if not lf:
            continue
        lab = pd.read_parquet(lf[0], columns=["bag_time_ns", "ctrl_steer", "ctrl_u", "ctrl_ok"])
        bt, cok = lab.bag_time_ns.to_numpy(np.int64), lab.ctrl_ok.to_numpy(bool)
        z = np.load(f)
        v, yaw, ns, ga, yr = z["v"].astype(np.float64), z["yaw"], z["bag_ns"], z["a"], z["yaw_rate"]
        ok = np.zeros(len(v), bool); ok[N_HIST + 1:len(v) - N_FUT - 1] = True
        if "clean" in z.files:
            ok &= z["clean"]
        idx = np.nonzero(ok)[0]
        if not len(idx):
            continue
        kap = np.where(np.abs(v[idx]) > 1.0, yr[idx] / np.maximum(np.abs(v[idx]), 1.0), 0.0)
        w = 1.0 + 4.0 * (ga[idx] < -2.0) + 2.0 * (np.abs(v[idx]) < 0.3) + 4.0 * (np.abs(kap) > 0.05)
        pick = rng.choice(idx, min(per_ep, len(idx)), replace=False, p=w / w.sum())

        def applied(i):
            j = int(np.searchsorted(bt, ns[i], side="right") - 1)
            if j < 0 or not cok[j]:
                return None
            k = int(round((ns[i] - bt[j]) / 5e7)); s, u = lab.ctrl_steer.iat[j], lab.ctrl_u.iat[j]
            return None if not 0 <= k < len(s) else (max(u[k], 0.0), max(-u[k], 0.0), -math.degrees(s[k]))
        keep, hist, tgt = [], [], []
        for i in pick:
            a0, a1, t1 = applied(i - 1), applied(i), applied(i + 1)
            if a0 and a1 and t1:
                keep.append(i); hist.append(a0 + a1); tgt.append(t1)
        if not keep:
            continue
        trajs = [ego_trajectory(v, yaw, int(i)) for i in keep]
        with torch.no_grad():
            act = space.traj_to_action(*(torch.cat([t[q] for t in trajs]).double() for q in range(4))).numpy()
        A = act[..., 0] * cfg["accel_std"] + cfg["accel_mean"]; K = act[..., 1] * cfg["curvature_std"] + cfg["curvature_mean"]
        for r, i in enumerate(keep):
            acc, brk, st, _ = ctl.command(Plan(0.0, A[r], K[r], v[i]), 0.0, v[i])
            cols["x"].append(np.r_[A[r, :N_STEPS], K[r, :N_STEPS], v[i], hist[r]])
            cols["y"].append(tgt[r]); cols["map"].append((acc, brk, st * 40.0))
            cols["ep"].append(ep); cols["v"].append(v[i]); cols["k1"].append(K[r, 1])
        if n % 100 == 0:
            print(n, len(cols["x"]), flush=True)
    np.savez(out, **{k: np.array(v) for k, v in cols.items()})
    print("rows", len(cols["x"]), "episodes", len(set(cols["ep"])))


def train(path, prefix):
    import torch
    d = np.load(path)
    X, Y, M, ep, v, k1 = d["x"].astype(np.float32), d["y"].astype(np.float32), d["map"], d["ep"], d["v"], d["k1"]
    eps = np.unique(ep); rng = np.random.default_rng(0); val_eps = set(rng.choice(eps, len(eps) // 5, replace=False))
    va = np.array([e in val_eps for e in ep]); tr_all = np.nonzero(~va)[0]
    tr_half = rng.choice(tr_all, len(tr_all) // 2, replace=False)
    print("rows %d (train %d, val %d from %d held-out episodes)" % (len(X), len(tr_all), va.sum(), len(val_eps)))

    def fit(tr, use_hist):
        cols = slice(None) if use_hist else slice(0, 2 * N_STEPS + 1)
        x = X[:, cols]; mu, sd = x[tr].mean(0), x[tr].std(0) + 1e-6
        torch.manual_seed(0)
        net = torch.nn.Sequential(torch.nn.Linear(x.shape[1], 128), torch.nn.SiLU(), torch.nn.Linear(128, 128), torch.nn.SiLU(),
                                  torch.nn.Linear(128, 3))
        opt = torch.optim.Adam(net.parameters(), 1e-3)
        xt, yt = torch.tensor((x[tr] - mu) / sd), torch.tensor(Y[tr])
        out = lambda h: torch.stack([torch.sigmoid(h[:, 0]), torch.sigmoid(h[:, 1]), 40.0 * torch.tanh(h[:, 2])], 1)
        for epoch in range(60):
            perm = torch.randperm(len(xt))
            for b in range(0, len(xt), 256):
                j = perm[b:b + 256]; p = out(net(xt[j]))
                loss = ((p[:, :2] - yt[j, :2]) ** 2).mean() + (((p[:, 2] - yt[j, 2]) / 40.0) ** 2).mean()
                opt.zero_grad(); loss.backward(); opt.step()
        with torch.no_grad():
            pred = out(net(torch.tensor((x - mu) / sd))).numpy()
        W = [m for m in net if isinstance(m, torch.nn.Linear)]
        weights = dict(mu=mu, sd=sd, **{f"w{i}": l.weight.detach().numpy() for i, l in enumerate(W)},
                       **{f"b{i}": l.bias.detach().numpy() for i, l in enumerate(W)}, hist=np.array(use_hist))
        return pred, weights

    def report(name, P):
        e = P[va] - Y[va]
        q = lambda x: "%.3f/%.3f" % (np.median(np.abs(x)), np.percentile(np.abs(x), 95))
        line = "%-22s steer %s deg  accel %s  brake %s" % (name, q(e[:, 2]), q(e[:, 0]), q(e[:, 1]))
        print(line)
        return e

    results = {"map": report("map (control.py)", M.astype(np.float32))}
    for tag, tr, hist in (("mlp 30k hist", tr_all, True), ("mlp 15k hist", tr_half, True), ("mlp 30k no-hist", tr_all, False),
                          ("mlp 15k no-hist", tr_half, False)):
        pred, w = fit(tr, hist)
        results[tag] = report(tag, pred)
        if tr is tr_all:
            np.savez(prefix + ("" if hist else "_nohist") + ".npz", **w)
    vv, kk = v[va], np.abs(k1[va])
    for title, bins, key in (("speed", SPEED_BINS, np.abs(vv)), ("curvature |k1|", CURV_BINS, kk)):
        print("by %s (steer deg / accel / brake, p50/p95):" % title)
        for name, lo, hi in bins:
            m = (key >= lo) & (key < hi)
            cells = []
            for tag in ("map", "mlp 30k hist", "mlp 30k no-hist"):
                e = results[tag][m]
                cells.append("%s %.2f/%.2f %.3f/%.3f %.3f/%.3f" % (tag.split()[0] + ("" if tag == "map" else "-" + tag.split()[-1]),
                             *(f(np.abs(e[:, c])) for c in (2, 0, 1) for f in (np.median, lambda x: np.percentile(x, 95)))))
            print("  %-9s n %5d | %s" % (name, m.sum(), " | ".join(cells)))


if __name__ == "__main__":
    if sys.argv[1] == "data":
        data(*sys.argv[2:7], **({"n_rows": int(sys.argv[7])} if len(sys.argv) > 7 else {}))
    else:
        train(sys.argv[2], sys.argv[3])
