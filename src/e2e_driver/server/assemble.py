"""Model-server side (python 3.12 venv of the training code): a request from the ROS node -> the raw sample dict that
morai_vla.data.MoraiDataset.sample returns for a training row, built with the training functions themselves:
  camera_frames  PIL decode of the step-0 JPEGs, as data.Episode.jpeg
  bev_frames     lidar_bev.pack_bev(scan k1, scan k0, pose k1, pose k0) and (k3, k2), as MoraiDataset.sample
  ego_history_*  data.ego_trajectory on the 16 history points (index 15 = t0; the future half is not used at inference)
Then processor.get_preprocess_fn(generation_mode=True) and processor.collate_fn make the model batch (tokenize()).
Needs the training code on sys.path (MORAI_VLA_TRAIN, e.g. ~/morai-student/train).
"""
import io

import numpy as np
import torch
from PIL import Image

from morai_vla.data import N_FUT, N_HIST, ego_trajectory
from morai_vla.lidar_bev import pack_bev
from morai_vla.processor import collate_fn, get_preprocess_fn

DROP = ("camera_frames", "bev_frames", "camera_times", "bev_times")      # morai_vla.stage1_eval: tokenized already


def jpeg_tensor(b):
    return torch.from_numpy(np.array(Image.open(io.BytesIO(b)).convert("RGB"))).permute(2, 0, 1)


def build_sample(req, key="live"):
    out = dict(camera_frames=[torch.stack([jpeg_tensor(b) for b in cam]) for cam in req["camera_jpegs"]],
               camera_times=np.asarray(req["camera_times"], np.float64), key=key, nav_text=str(req["nav_text"]),
               lidar_dropped="scan_height" not in req, cot=None)
    if "scan_height" in req:
        sc = [np.stack([h, d]) for h, d in zip(req["scan_height"], req["scan_density"])]
        pose = [tuple(p) for p in np.asarray(req["scan_pose"], np.float64)]
        out["bev_frames"] = torch.from_numpy(np.stack([pack_bev(sc[n + 1], sc[n], pose[n + 1], pose[n]) for n in (0, 2)]))
        out["bev_times"] = np.asarray(req["bev_times"], np.float64)
    v = np.r_[np.asarray(req["ego_v"], np.float64), np.zeros(N_FUT)]
    yaw = np.r_[np.asarray(req["ego_yaw"], np.float64), np.full(N_FUT, float(req["ego_yaw"][-1]))]
    hx, hr, fx, fr = ego_trajectory(v, yaw, N_HIST - 1)
    out.update(ego_history_xyz=hx, ego_history_rot=hr, ego_future_xyz=torch.zeros_like(fx), ego_future_rot=torch.zeros_like(fr))
    return out


def tokenize(sample, model_config):
    """raw sample -> the collated batch the model's sample_trajectories takes (CPU tensors)."""
    d = dict(sample)
    td = get_preprocess_fn(model_config, generation_mode=True)(data=d)          # also sets label_components, generation_mode
    item = {k: v for k, v in d.items() if k not in DROP}
    item["tokenized_data"] = td
    return collate_fn([item], model_config=model_config)
