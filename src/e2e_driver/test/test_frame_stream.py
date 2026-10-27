"""Frames streamed ahead of requests (assemble.FrameStore / warm, the engine's tokenize(fast=True)) vs the uncached
training path, bit for bit, on recorded requests replayed as the node now sends them: each camera JPEG and LiDAR scan
streamed first, then the request naming them by stamp. Per request, in order (so later ones reuse earlier records):
  resolve   the stamp request comes back as the recorded request (bytes and arrays);
  engine    cpu_prepare's batch (input_ids, attention_mask, grid, lidar flags, ego history, labels mask), per-image bf16
            pixels (cat), ViT-cache fingerprints and M-RoPE layout equal the training path's (processor without caches,
            fingerprints / layout / bf16 of its pixel_values), warmed or not;
  slow      tokenize() without fast (HF / fake backends) on the stand-in frames gives the training path's fp32 pixels.
Then the CPU time per request: the old steady state (every image cached, full pixel tensors) vs warmed records, the
warm thread's cost per frame; and model_server (fake backend, --preprocess-config) over a socket: frames streamed, stamp
requests answered, --dump keeps the full request, a request naming a frame never streamed drops the connection.
With DELIVER=<deliver dir> on a GPU: the engine's actions on the old batch (one pixel tensor, pinned per request) and on
the records (per-image list) are equal for the same noise (EngineBackend, ViT cache cleared before each run).

    <venv>/bin/python test/test_frame_stream.py <dir with the model's config.json> [dir of req_*.bin, default assets/warmup]
    GPU: DELIVER=$V/models/deliver ... test/test_frame_stream.py $V/models/deliver/final [dump dir]
    Mac: ~/claude-bench/venv-rl/bin/python test/test_frame_stream.py ~/claude-bench/rl-tiny/deliver/final
"""
import os
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "server"), str(HERE.parent / "src"), str(HERE.parent / "server" / "vendor")]
os.environ.setdefault("HF_HUB_OFFLINE", "1")

import numpy as np  # noqa: E402
import torch  # noqa: E402

import assemble  # noqa: E402
from assemble import FrameStore, build_sample, tokenize, warm  # noqa: E402
from e2e_driver import wire  # noqa: E402
from fast_engine import fingerprints, layout  # noqa: E402
from morai_vla.model import load_config  # noqa: E402
from prep_proc import cpu_prepare  # noqa: E402


def stream(req, t_scan):
    """the node's messages for a recorded request: frames (stamps t0 + camera_times; scans at t_scan) + the stamp request."""
    t0, msgs = float(req["t0"]), []
    stamps = [[t0 + float(x) for x in row] for row in req["camera_times"]]
    for i, (ts, jpegs) in enumerate(zip(stamps, req["camera_jpegs"])):
        msgs += [dict(frame="cam", cam=i, t=t, jpeg=j) for t, j in zip(ts, jpegs)]
    msgs += [dict(frame="scan", t=t, height=h, density=d, pose=p)
             for t, h, d, p in zip(t_scan, req["scan_height"], req["scan_density"], req["scan_pose"])]
    slim = {k: v for k, v in req.items() if k not in ("camera_jpegs", "scan_height", "scan_density", "scan_pose")}
    return [wire.unpack(wire.pack(m)) for m in msgs], wire.unpack(wire.pack(dict(slim, camera_stamps=stamps, scan_stamps=t_scan)))


def clear():
    for c in (assemble.REC, assemble._JPEG, assemble._BEV):
        c.clear()


def reference(req, cfg):
    """the training path: frames decoded, processor without caches -> batch, fp32 pixels, fingerprints, layout."""
    clear()
    s = build_sample(req, key="ref")
    s.pop("_image_keys"), s.pop("_recs")
    b = tokenize(s, cfg)
    td = b["tokenized_data"]
    clear()
    return b, td["pixel_values"], fingerprints(td["pixel_values"], td["image_grid_thw"]), layout(td["input_ids"], td["image_grid_thw"], IMG, MERGE)


def same(a, b, path="batch"):
    """nested dict / list / tensor / array equality -> list of differing paths."""
    if isinstance(a, dict):
        return [p for k in set(a) | set(b) for p in (same(a.get(k), b.get(k), f"{path}.{k}") if k in a and k in b else [f"{path}.{k} missing"])]
    if isinstance(a, (list, tuple)) and not isinstance(a, str):
        return [p for i, (x, y) in enumerate(zip(a, b)) for p in same(x, y, f"{path}[{i}]")] + ([f"{path} len"] if len(a) != len(b) else [])
    if torch.is_tensor(a):
        return [] if torch.is_tensor(b) and a.dtype == b.dtype and torch.equal(a, b) else [path]
    if isinstance(a, np.ndarray):
        return [] if isinstance(b, np.ndarray) and a.dtype == b.dtype and np.array_equal(a, b) else [path]
    return [] if a == b else [path]


IMG, MERGE = 151655, 2                                    # Qwen3-VL image token, spatial merge


def main():
    cfg = load_config(Path(os.path.expanduser(sys.argv[1])), "Qwen/Qwen3-VL-4B-Instruct", "sdpa")
    reqdir = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE.parent / "assets" / "warmup"
    reqs = [wire.unpack(f.read_bytes()) for f in sorted(reqdir.glob("req_*.bin"))]
    assert reqs, reqdir
    refs = [reference(r, cfg) for r in reqs]
    for warmed in (True, False):
        clear()
        store = FrameStore()
        for n, (req, (b_ref, pv_ref, keys_ref, lay_ref)) in enumerate(zip(reqs, refs)):
            t_scan = [float(req["t0"]) - 0.3 + 0.1 * j for j in range(4)]                     # synthetic, consistent stamps
            if n:                                                                             # consecutive requests: scans move on one
                t_scan = [t + 0.1 * n for t in t_scan]
            msgs, slim = stream(req, t_scan)
            for m in msgs:
                job = store.add(m)
                if warmed and job is not None:
                    warm(cfg, job)
            full = store.resolve(slim)
            bad = same({k: req[k] for k in req}, {k: full[k] for k in req}, "request")
            assert not bad, bad
            s = build_sample(full, key=f"new/{n}")
            stand_ins = sum(f.shape[1:] == (0,) for f in s["camera_frames"])
            (b, keys, lay), _ = cpu_prepare(s, cfg, IMG, MERGE)
            td, td_ref = b["tokenized_data"], b_ref["tokenized_data"]
            assert isinstance(td["pixel_values"], list), type(td["pixel_values"])
            assert torch.equal(torch.cat(td["pixel_values"]), pv_ref.to(torch.bfloat16)), "pixels"
            assert keys == keys_ref, "fingerprints"
            assert all(torch.equal(x, y) for x, y in zip(lay, lay_ref)), "layout"
            bad = same({k: v for k, v in td.items() if k != "pixel_values"}, {k: v for k, v in td_ref.items() if k != "pixel_values"})
            bad += same({k: v for k, v in b.items() if k not in ("tokenized_data", "key")}, {k: v for k, v in b_ref.items() if k not in ("tokenized_data", "key")})
            assert not bad, bad
            s2 = build_sample(full, key=f"slow/{n}")                                            # HF / fake backends
            assert torch.equal(tokenize(s2, cfg)["tokenized_data"]["pixel_values"], pv_ref), "slow-path pixels"
            print(f"request {n} warmed={warmed}: equal (camera groups as stand-ins {stand_ins}/4, bev "
                  f"{'stand-in' if s['bev_frames'].shape[1:] == (0,) else 'packed'})")

    # CPU time: the old steady state (all images cached, full pixel tensors) vs warmed records
    assemble_old = assemble.build_sample

    def build_old(req, key="live"):                    # no stand-ins: frames decoded and stacked as before
        held = assemble._held
        assemble._held = lambda *a: None
        try:
            return assemble_old(req, key)
        finally:
            assemble._held = held
    clear()
    t_old, t_new = [], []
    for _ in range(4):
        for req in reqs:
            t = time.perf_counter(); s = build_old(req); s["_recs"] = {}
            b = tokenize(s, cfg); td = b["tokenized_data"]
            fingerprints(td["pixel_values"], td["image_grid_thw"]); layout(td["input_ids"], td["image_grid_thw"], IMG, MERGE)
            td["pixel_values"].to(torch.bfloat16)
            t_old.append(time.perf_counter() - t)
            t = time.perf_counter(); cpu_prepare(build_sample(req, key="new"), cfg, IMG, MERGE)
            t_new.append(time.perf_counter() - t)
    print("CPU ms per request (steady state, %d runs): old %.1f, records %.1f" % (len(t_old) - 2, 1e3 * np.median(t_old[2:]), 1e3 * np.median(t_new[2:])))
    clear()                                            # the warm thread's cost per streamed frame (all new)
    w = {"cam": [], "bev": []}
    store = FrameStore()
    for n, req in enumerate(reqs):
        for m in stream(req, [float(req["t0"]) - 0.3 + 0.1 * (j + n) for j in range(4)])[0]:
            job = store.add(m)
            if job is not None:
                t = time.perf_counter(); warm(cfg, job); w["cam" if "jpeg" in job else "bev"].append(time.perf_counter() - t)
    print("warm ms per frame: camera %.1f, BEV %.1f" % (1e3 * np.median(w["cam"]), 1e3 * np.median(w["bev"])))
    server(sys.argv[1], reqs)
    if os.environ.get("DELIVER"):
        engine_check(os.environ["DELIVER"], reqs, build_old)
    print("frame stream ok")


def engine_check(deliver, reqs, build_old):
    from backends import EngineBackend
    be = EngineBackend(deliver, seed=0)
    fe, cfg = be.fast, be.model.config
    rows = []
    for n, req in enumerate(reqs[:16]):
        s = build_old(req, key="old"); s["_recs"] = {}
        b = tokenize(s, cfg); td = b["tokenized_data"]                     # the old cpu_prepare
        keys = fingerprints(td["pixel_values"], td["image_grid_thw"])
        lay = layout(td["input_ids"], td["image_grid_thw"], fe.image_token, fe.vlm.visual.spatial_merge_size)
        td["pixel_values"] = td["pixel_values"].to(torch.bfloat16)
        old = be.pin(((b, keys, lay), {}))
        new = be.prepare(build_sample(req, key="new"))
        acts = []
        for p in (old, new):
            be.reset(); be.n = n
            acts.append(torch.from_numpy(np.stack([be.run(s, p)[k] for k in ("accel", "kappa")])))
        rows.append(torch.equal(*acts))
    assert all(rows), rows
    print("engine ok: actions equal on %d requests (old pixel tensor vs per-image records)" % len(rows))


def server(config_dir, reqs):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]
    dump = Path(tempfile.mkdtemp())
    p = subprocess.Popen([sys.executable, str(HERE.parent / "server" / "model_server.py"), "--backend", "fake", "--port", str(port),
                          "--preprocess-config", os.path.expanduser(config_dir), "--dump", str(dump)],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        for _ in range(600):
            try:
                c = socket.create_connection(("127.0.0.1", port), timeout=60); break
            except OSError:
                time.sleep(0.2)
        for n, req in enumerate(reqs):
            msgs, slim = stream(req, [float(req["t0"]) - 0.3 + 0.1 * (j + n) for j in range(4)])
            for m in msgs:
                wire.send(c, m)
            wire.send(c, dict(slim, id=n))
            r = wire.recv(c)
            assert r["id"] == n and r["timing"]["prefix_tokens"] > 1000, r
            kept = wire.unpack((dump / f"req_{n:06d}.bin").read_bytes())
            bad = same({k: req[k] for k in req}, {k: kept[k] for k in req}, "dump")
            assert not bad, bad
        wire.send(c, dict(slim, id=9, camera_stamps=[[1.5] * 4] * 4))                      # never streamed
        try:
            wire.recv(c); raise AssertionError("answered a request naming unknown frames")
        except (ConnectionError, OSError):
            pass
        print("model_server ok: %d stamp requests answered, dumps are the full requests, unknown frame -> dropped" % len(reqs))
    finally:
        p.terminate()
        out = p.communicate(timeout=10)[0]
        if sys.exc_info()[0] is not None:
            print(out)
    assert "model_server warm" not in out, out[-3000:]                 # the warm thread made every record


if __name__ == "__main__":
    main()
