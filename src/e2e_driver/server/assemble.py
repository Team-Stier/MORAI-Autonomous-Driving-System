"""Model-server side (python 3.12 venv of the training code): a request from the ROS node -> the raw sample dict that
morai_vla.data.MoraiDataset.sample returns for a training row, built with the training functions themselves:
  camera_frames  PIL decode of the step-0 JPEGs, as data.Episode.jpeg
  bev_frames     lidar_bev.pack_bev(scan k1, scan k0, pose k1, pose k0) and (k3, k2), as MoraiDataset.sample
  ego_history_*  data.ego_trajectory on the 16 history points (index 15 = t0; the future half is not used at inference)
Then processor.get_preprocess_fn(generation_mode=True) and processor.collate_fn make the model batch (tokenize()).
Needs the training code on sys.path (MORAI_VLA_TRAIN, e.g. ~/morai-student/train).

Live, consecutive requests share 12 of their 16 camera frames and often their BEVs, so the per-image work is cached by
content (blake2b of the JPEG / of the BEV inputs): the JPEG decode, pack_bev, and the image processor's output per image
(Rec in REC, through CachedImageProcessor). The training functions still build the whole prompt; test/offline_compare.py
checks the cached path bit for bit against the uncached training path.

Frames ahead of requests: the node streams each step-0 JPEG and LiDAR scan as soon as it has it and names them by stamp
in its requests (FrameStore, one per connection, turns a request back into the full one). A worker thread per connection
makes each streamed image's record before the request that uses it arrives (warm). build_sample then hands preprocess an
empty stand-in for a camera group (or the BEV pair) whose records all exist, and the engine's tokenize(fast=True) takes
the pixels and ViT-cache fingerprints per image from the records: no per-request decode, stack, cat, cast, hash or pin of
all 18 images (~35 MB of fp32 pixels, ~25 ms a request before). test/test_frame_stream.py checks it bit for bit against
the uncached training path.
"""
import hashlib
import io
import threading
from collections import OrderedDict

import numpy as np
import torch
from PIL import Image

from morai_vla.data import N_FUT, N_HIST, ego_trajectory
from morai_vla.lidar_bev import pack_bev
from morai_vla.processor import BEV, CAMERAS, collate_fn, get_preprocess_fn, get_processor

DROP = ("camera_frames", "bev_frames", "camera_times", "bev_times")      # morai_vla.stage1_eval: tokenized already
FAST = False                     # the engine sets it: records' bf16 rows page-locked (made by warm, read every request)
OPT = lambda w, h: (("do_rescale", False), ("max_pixels", w * h), ("min_pixels", w * h))   # preprocess's ip kwargs, sorted


class LRU(OrderedDict):
    """shared by the prepare and warm threads: dict changes under a lock, values made outside it."""

    def __init__(self, size):
        super().__init__(); self.size, self.lock = size, threading.Lock()

    def peek(self, key):
        with self.lock:
            v = self.get(key)
            if v is not None:
                self.move_to_end(key)
            return v

    def put(self, key, v):
        with self.lock:
            self[key] = v; self.move_to_end(key)
            while len(self) > self.size:
                self.popitem(last=False)
        return v

    def get_or(self, key, make):
        v = self.peek(key)
        return v if v is not None else self.put(key, make())


class Rec:
    """one image's image-processor result (its pixel_values rows, grid row, the call's options); fast(): + its bf16 rows
    (page-locked with FAST) and ViT-cache fingerprint (fast_engine.fingerprints of the image alone), made once."""
    __slots__ = ("pv", "grid", "opt", "bf", "key")

    def __init__(self, pv, grid, opt):
        self.pv, self.grid, self.opt, self.bf, self.key = pv, grid, opt, None, None

    def fast(self):
        if self.bf is None:
            from fast_engine import fingerprints
            self.key = fingerprints(self.pv, self.grid[None])[0]
            bf = self.pv.to(torch.bfloat16)                                 # the ViT input dtype
            self.bf = bf.pin_memory() if FAST else bf
        return self


_JPEG, _BEV, REC = LRU(128), LRU(32), LRU(160)


def digest(*parts):
    h = hashlib.blake2b(digest_size=16)
    for p in parts:
        h.update(p if isinstance(p, (bytes, bytearray)) else np.ascontiguousarray(p).tobytes())
    return h.digest()


def jpeg_tensor(b):
    return torch.from_numpy(np.array(Image.open(io.BytesIO(b)).convert("RGB"))).permute(2, 0, 1)


def _held(keys, w, h):
    """the records of one image group if all exist with this group's options (they stay alive with the request)."""
    recs = [REC.peek(k) for k in keys]
    return recs if all(r is not None and r.opt == OPT(w, h) for r in recs) else None


def build_sample(req, key="live"):
    keys = [[digest(b) for b in cam] for cam in req["camera_jpegs"]]
    held, frames = {}, []
    for (_, w, h), ks, cam in zip(CAMERAS, keys, req["camera_jpegs"]):
        recs = _held(ks, w, h)
        if recs:                                         # preprocess only counts the frames: records give the rest
            held.update(zip(ks, recs)); frames.append(torch.empty(len(cam), 0, dtype=torch.uint8))
        else:
            frames.append(torch.stack([_JPEG.get_or(k, lambda b=b: jpeg_tensor(b)) for k, b in zip(ks, cam)]))
    out = dict(camera_frames=frames, camera_times=np.asarray(req["camera_times"], np.float64), key=key,
               nav_text=str(req["nav_text"]), lidar_dropped="scan_height" not in req, cot=None)
    if "scan_height" in req:
        sc = [np.stack([h, d]) for h, d in zip(req["scan_height"], req["scan_density"])]
        pose = [tuple(p) for p in np.asarray(req["scan_pose"], np.float64)]
        bk = [digest(sc[n + 1], sc[n], np.asarray(pose[n + 1]), np.asarray(pose[n])) for n in (0, 2)]
        recs = _held(bk, BEV[1], BEV[2])
        if recs:
            held.update(zip(bk, recs)); out["bev_frames"] = torch.empty(2, 0, dtype=torch.uint8)
        else:
            out["bev_frames"] = torch.from_numpy(np.stack([_BEV.get_or(k, lambda n=n: pack_bev(sc[n + 1], sc[n], pose[n + 1], pose[n]))
                                                          for k, n in zip(bk, (0, 2))]))
        out["bev_times"] = np.asarray(req["bev_times"], np.float64)
        keys.append(bk)
    v = np.r_[np.asarray(req["ego_v"], np.float64), np.zeros(N_FUT)]
    yaw = np.r_[np.asarray(req["ego_yaw"], np.float64), np.full(N_FUT, float(req["ego_yaw"][-1]))]
    hx, hr, fx, fr = ego_trajectory(v, yaw, N_HIST - 1)
    out.update(ego_history_xyz=hx, ego_history_rot=hr, ego_future_xyz=torch.zeros_like(fx), ego_future_rot=torch.zeros_like(fr))
    out["_image_keys"], out["_recs"] = keys, held   # per image-processor call (cameras in order, then the BEV pair)
    return out


class CachedImageProcessor:
    """the processor's image processor with per-image records (REC). Qwen2VLImageProcessorFast handles every image on
    its own (rescale, normalize and patchify are per image; the min = max pixel bounds keep the size), so an image's
    pixel_values rows and grid row are the same whether it is processed alone or with others. tokenize() queues one key
    list per call MoraiProcessor.preprocess makes, and the records build_sample holds; a call without queued keys goes
    straight through (training path). fast: pixel_values come back empty (N, 0) and the records used go to `used`."""

    def __init__(self, ip):
        self._ip, self.keys, self.held, self.fast, self.used = ip, [], {}, False, []

    def __getattr__(self, name):
        return getattr(self._ip, name)

    def __call__(self, images, **kw):
        if not self.keys:
            return self._ip(images=images, **kw)
        keys, opt = self.keys.pop(0), tuple(sorted(kw.items()))
        recs = [self.held.get(k) or REC.peek(k) for k in keys]
        recs = [r if r is not None and r.opt == opt else None for r in recs]
        miss = [i for i, r in enumerate(recs) if r is None]
        if miss:
            out = self._ip(images=images[miss], **kw)
            for i, pv, g in zip(miss, out["pixel_values"].split(out["image_grid_thw"].prod(-1).tolist()), out["image_grid_thw"]):
                recs[i] = REC.put(keys[i], Rec(pv, g, opt))
        grid = torch.stack([r.grid for r in recs])
        if self.fast:
            self.used += recs
            return {"pixel_values": torch.empty(sum(r.pv.shape[0] for r in recs), 0), "image_grid_thw": grid}
        return {"pixel_values": torch.cat([r.pv for r in recs]), "image_grid_thw": grid}


def processor(model_config):
    proc = get_processor(model_config).processor
    if not isinstance(proc.image_processor, CachedImageProcessor):
        proc.image_processor = CachedImageProcessor(proc.image_processor)
    return proc


def tokenize(sample, model_config, fast=False):
    """raw sample -> the collated batch the model's sample_trajectories takes (CPU tensors). fast (the engine): -> (batch,
    the records of its images in order) with tokenized_data pixel_values left empty; records only for a live sample."""
    d = dict(sample)
    keys, held = d.pop("_image_keys", None), d.pop("_recs", {})
    ip = processor(model_config).image_processor
    ip.keys, ip.held, ip.fast, ip.used = ([list(k) for k in keys] if keys else []), held, bool(fast and keys), []
    try:
        td = get_preprocess_fn(model_config, generation_mode=True)(data=d)      # also sets label_components, generation_mode
    finally:
        used = ip.used
        ip.keys, ip.held, ip.fast, ip.used = [], {}, False, []
    item = {k: v for k, v in d.items() if k not in DROP}
    item["tokenized_data"] = td
    b = collate_fn([item], model_config=model_config)
    return (b, used) if fast else b


def _record(ip, k, frames, w, h):
    """the record of image k (frames(): its (1, C, H, W) frame, only called when missing), made as preprocess makes it."""
    r = REC.peek(k)
    if r is None or r.opt != OPT(w, h):
        out = ip._ip(images=frames().float() / 255.0, do_rescale=False, min_pixels=w * h, max_pixels=w * h)
        r = REC.put(k, Rec(out["pixel_values"], out["image_grid_thw"][0], OPT(w, h)))
    return r.fast() if FAST else r


def warm(model_config, job):
    """a FrameStore.add job ahead of the request: the record of a streamed camera JPEG, or of the BEV of a scan and the one
    received before it (the pair a request with consecutive scans uses), keyed exactly as build_sample keys them."""
    ip = processor(model_config).image_processor
    if "jpeg" in job:
        _, w, h = CAMERAS[int(job["cam"])]
        k, b = digest(job["jpeg"]), job["jpeg"]
        return _record(ip, k, lambda: _JPEG.get_or(k, lambda: jpeg_tensor(b))[None], w, h)
    (h1, d1, p1), (h0, d0, p0) = job["bev"]
    sc1, sc0, p1, p0 = np.stack([h1, d1]), np.stack([h0, d0]), tuple(p1), tuple(p0)
    k = digest(sc1, sc0, np.asarray(p1), np.asarray(p0))
    return _record(ip, k, lambda: torch.from_numpy(_BEV.get_or(k, lambda: pack_bev(sc1, sc0, p1, p0))[None]), BEV[1], BEV[2])


class FrameStore:
    """what one node streamed ahead of its requests (model_server: one per connection): step-0 JPEGs by (camera index,
    stamp) and LiDAR scans by stamp, the newest `size` / `scans` of each kept by arrival; a frame sent again (the node
    resends after a reset) replaces the old one."""

    def __init__(self, size=256, scans=64):
        self.size, self.n_scans, self.cams, self.scans, self.lock = size, scans, {}, OrderedDict(), threading.Lock()

    @staticmethod
    def _keep(d, k, v, size):
        d[k] = v; d.move_to_end(k)
        while len(d) > size:
            d.popitem(last=False)

    def add(self, m):
        """a streamed frame {frame: cam, cam, t, jpeg} or {frame: scan, t, height, density, pose} -> its warm() job or None."""
        with self.lock:
            if m["frame"] == "cam":
                self._keep(self.cams.setdefault(int(m["cam"]), OrderedDict()), m["t"], m["jpeg"], self.size)
                return m
            prev = next(reversed(self.scans.values()), None)
            new = (m["height"], m["density"], np.asarray(m["pose"], np.float64))
            self._keep(self.scans, m["t"], new, self.n_scans)
            return None if prev is None else dict(bev=(new, prev))

    def resolve(self, req):
        """a request naming its frames by stamp only (camera_stamps, scan_stamps; a full one passes as it is) -> the full
        request the node used to send; KeyError: a frame it never streamed."""
        with self.lock:
            if "camera_jpegs" not in req and "camera_stamps" in req:
                req["camera_jpegs"] = [[self.cams[i][t] for t in ts] for i, ts in enumerate(req.pop("camera_stamps"))]
            if "scan_height" not in req and "scan_stamps" in req:
                sc = [self.scans[t] for t in req.pop("scan_stamps")]
                req.update(scan_height=np.stack([s[0] for s in sc]), scan_density=np.stack([s[1] for s in sc]),
                           scan_pose=np.stack([s[2] for s in sc]))
        return req
