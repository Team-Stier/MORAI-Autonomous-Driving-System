"""Model-server side (python 3.12 venv of the training code): a request from the ROS node -> the raw sample dict that
morai_vla.data.MoraiDataset.sample returns for a training row, built with the training functions themselves:
  camera_frames  PIL decode of the step-0 JPEGs, as data.Episode.jpeg
  bev_frames     lidar_bev.pack_bev(scan k1, scan k0, pose k1, pose k0) and (k3, k2), as MoraiDataset.sample
  ego_history_*  data.ego_trajectory on the 16 history points (index 15 = t0; the future half is not used at inference)
Then processor.get_preprocess_fn(generation_mode=True) and processor.collate_fn make the model batch (tokenize()).
Needs the training code on sys.path (MORAI_VLA_TRAIN, e.g. ~/morai-student/train).

Live, consecutive requests share 12 of their 16 camera frames and often their BEVs, so the per-image work is cached by
content (blake2b of the JPEG / of the BEV inputs): the JPEG decode, pack_bev, and the image processor's output per image
(CachedImageProcessor). The training functions still build the whole prompt; test/offline_compare.py checks the cached
path bit for bit against the uncached training path.
"""
import hashlib
import io
from collections import OrderedDict

import numpy as np
import torch
from PIL import Image

from morai_vla.data import N_FUT, N_HIST, ego_trajectory
from morai_vla.lidar_bev import pack_bev
from morai_vla.processor import collate_fn, get_preprocess_fn, get_processor

DROP = ("camera_frames", "bev_frames", "camera_times", "bev_times")      # morai_vla.stage1_eval: tokenized already


class LRU(OrderedDict):
    def __init__(self, size):
        super().__init__(); self.size = size

    def get_or(self, key, make):
        if key in self:
            self.move_to_end(key); return self[key]
        v = self[key] = make()
        while len(self) > self.size:
            self.popitem(last=False)
        return v


_JPEG, _BEV = LRU(128), LRU(32)


def digest(*parts):
    h = hashlib.blake2b(digest_size=16)
    for p in parts:
        h.update(p if isinstance(p, (bytes, bytearray)) else np.ascontiguousarray(p).tobytes())
    return h.digest()


def jpeg_tensor(b):
    return torch.from_numpy(np.array(Image.open(io.BytesIO(b)).convert("RGB"))).permute(2, 0, 1)


def build_sample(req, key="live"):
    keys = [[digest(b) for b in cam] for cam in req["camera_jpegs"]]
    out = dict(camera_frames=[torch.stack([_JPEG.get_or(k, lambda b=b: jpeg_tensor(b)) for k, b in zip(ks, cam)])
                              for ks, cam in zip(keys, req["camera_jpegs"])],
               camera_times=np.asarray(req["camera_times"], np.float64), key=key, nav_text=str(req["nav_text"]),
               lidar_dropped="scan_height" not in req, cot=None)
    if "scan_height" in req:
        sc = [np.stack([h, d]) for h, d in zip(req["scan_height"], req["scan_density"])]
        pose = [tuple(p) for p in np.asarray(req["scan_pose"], np.float64)]
        bk = [digest(sc[n + 1], sc[n], np.asarray(pose[n + 1]), np.asarray(pose[n])) for n in (0, 2)]
        out["bev_frames"] = torch.from_numpy(np.stack([_BEV.get_or(k, lambda n=n: pack_bev(sc[n + 1], sc[n], pose[n + 1], pose[n]))
                                                      for k, n in zip(bk, (0, 2))]))
        out["bev_times"] = np.asarray(req["bev_times"], np.float64)
        keys.append(bk)
    v = np.r_[np.asarray(req["ego_v"], np.float64), np.zeros(N_FUT)]
    yaw = np.r_[np.asarray(req["ego_yaw"], np.float64), np.full(N_FUT, float(req["ego_yaw"][-1]))]
    hx, hr, fx, fr = ego_trajectory(v, yaw, N_HIST - 1)
    out.update(ego_history_xyz=hx, ego_history_rot=hr, ego_future_xyz=torch.zeros_like(fx), ego_future_rot=torch.zeros_like(fr))
    out["_image_keys"] = keys                     # per image-processor call (cameras in order, then the BEV pair)
    return out


class CachedImageProcessor:
    """the processor's image processor with a per-image result cache. Qwen2VLImageProcessorFast handles every image on
    its own (rescale, normalize and patchify are per image; the min = max pixel bounds keep the size), so an image's
    pixel_values rows and grid row are the same whether it is processed alone or with others. tokenize() queues one key
    list per call MoraiProcessor.preprocess makes; a call without queued keys goes straight through (training path)."""

    def __init__(self, ip, size=96):
        self._ip, self.cache, self.keys = ip, LRU(size), []

    def __getattr__(self, name):
        return getattr(self._ip, name)

    def __call__(self, images, **kw):
        if not self.keys:
            return self._ip(images=images, **kw)
        keys, opt = self.keys.pop(0), tuple(sorted(kw.items()))
        miss = [i for i, k in enumerate(keys) if (k, opt) not in self.cache]
        if miss:
            out = self._ip(images=images[miss], **kw)
            for i, pv, g in zip(miss, out["pixel_values"].split(out["image_grid_thw"].prod(-1).tolist()), out["image_grid_thw"]):
                self.cache.get_or((keys[i], opt), lambda pv=pv, g=g: (pv, g))
        res = [self.cache.get_or((k, opt), None) for k in keys]
        return {"pixel_values": torch.cat([p for p, _ in res]), "image_grid_thw": torch.stack([g for _, g in res])}


def tokenize(sample, model_config):
    """raw sample -> the collated batch the model's sample_trajectories takes (CPU tensors)."""
    d = dict(sample)
    keys = d.pop("_image_keys", None)
    proc = get_processor(model_config).processor
    if not isinstance(proc.image_processor, CachedImageProcessor):
        proc.image_processor = CachedImageProcessor(proc.image_processor)
    proc.image_processor.keys = [list(k) for k in keys] if keys else []
    try:
        td = get_preprocess_fn(model_config, generation_mode=True)(data=d)      # also sets label_components, generation_mode
    finally:
        proc.image_processor.keys = []
    item = {k: v for k, v in d.items() if k not in DROP}
    item["tokenized_data"] = td
    return collate_fn([item], model_config=model_config)
