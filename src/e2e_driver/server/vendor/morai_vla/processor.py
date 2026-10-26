"""Alpamayo-1.5 chat format for MORAI (MODEL_PLAN.md 2.2-2.5), shared by teacher and student.

Differences from the recipe's QwenProcessor (whose tokenizer setup and collate_fn are reused as is):
- per-camera min/max pixels = the camera's own size, so step-0 images are never re-scaled (the 1.5 default
  min_pixels 163840 would upscale every MORAI camera);
- `<-0.08 seconds>` time text (real tau offset) before every image instead of `frame k`;
- 2 LiDAR BEV images after the cameras, under their own name, routed to the LiDAR ViT via `image_is_lidar`;
- CoC only when the sample carries one (Stage 1-C keyframes); the prompt follows it, as the recipe's cot component.
Sequence: system | cam name, (time, image) x4 for Front, Left, Right, Cam7 | BEV name, (time, BEV) x2 | history | route | prompt
          || [CoC] <traj_future_start> 128 tokens <traj_future_end>
"""
from functools import partial

import torch
from alpamayo.chat_template import components as C
from alpamayo.processor.qwen_processor import QwenProcessor

CAMERAS = (("Front camera", 448, 256), ("Front left camera", 352, 256), ("Front right camera", 352, 256),
           ("Front upward camera", 256, 192))                 # (name, W, H); left/right look 70 deg out, Cam7 10 deg up
BEV = ("LiDAR BEV", 160, 320)
LABEL_COMPONENTS = ["cot", "traj_future"]


def time_text(t):
    return f"<{round(float(t), 2) + 0.0:.2f} seconds>"        # + 0.0 turns -0.00 into 0.00


class MoraiProcessor(QwenProcessor):
    def preprocess(self, data, tokens_hist, tokens_fut, generation_mode=False, generate_cot=False):
        proc = self.processor
        txt = lambda s: {"type": "text", "text": s}
        user, groups = [], []
        for (name, w, h), frames, times in zip(CAMERAS, data["camera_frames"], data["camera_times"]):
            user.append(txt(f"{name}: "))
            for t in times: user += [txt(time_text(t)), {"type": "image"}]
            groups.append((frames, w, h, False))
        if data.get("bev_frames") is not None:
            user.append(txt(f"{BEV[0]}: "))
            for t in data["bev_times"]: user += [txt(time_text(t)), {"type": "image"}]
            groups.append((data["bev_frames"], BEV[1], BEV[2], True))
        user += C.construct_traj_history(tokens_hist) + C.construct_route(data)
        cot = data.get("cot")
        comps = (["cot"] if cot or (generation_mode and generate_cot) else []) + ["traj_future"]
        user += C.construct_user_prompt(comps, comps, False)
        if generation_mode:
            assistant = C.construct_cot(data, ask_for_component=True) if generate_cot else C.construct_traj_future(tokens_fut, ask_for_component=True)
        else:
            assistant = (C.construct_cot(data) if cot else []) + C.construct_traj_future(tokens_fut)
        messages = [{"role": "system", "content": C.construct_system_prompt()}, {"role": "user", "content": user},
                    {"role": "assistant", "content": assistant}]
        text = proc.apply_chat_template(messages, tokenize=False, add_generation_prompt=False, continue_final_message=generation_mode)

        ip = proc.image_processor; patch = ip.patch_size
        pv, thw, lidar = [], [], []
        for frames, w, h, is_lidar in groups:
            out = ip(images=frames.float() / 255.0, do_rescale=False, min_pixels=w * h, max_pixels=w * h)
            assert (out["image_grid_thw"][:, 1:] == torch.tensor([h // patch, w // patch])).all(), (out["image_grid_thw"], w, h)
            pv.append(out["pixel_values"]); thw.append(out["image_grid_thw"]); lidar += [is_lidar] * len(frames)
        thw = torch.cat(thw)
        merge = ip.merge_size ** 2
        parts = text.split(proc.image_token)                       # expand each image placeholder to its token count
        assert len(parts) == len(thw) + 1
        text = parts[0] + "".join(proc.image_token * int(n // merge) + p for n, p in zip(thw.prod(-1), parts[1:]))
        data["label_components"], data["generation_mode"] = LABEL_COMPONENTS, generation_mode
        return {"text": text, "pixel_values": torch.cat(pv), "image_grid_thw": thw, "image_is_lidar": torch.tensor(lidar)}


_PROCESSORS = {}


def get_processor(model_config):
    key = (model_config.vlm_name_or_path, model_config.traj_vocab_size)
    if key not in _PROCESSORS:
        _PROCESSORS[key] = MoraiProcessor(vlm_name_or_path=model_config.vlm_name_or_path, traj_vocab_size=model_config.traj_vocab_size)
    return _PROCESSORS[key]


def get_preprocess_fn(model_config, generation_mode=False, generate_cot=False):
    """hydra target for MoraiDataset.vla_preprocess_args."""
    return partial(get_processor(model_config).preprocess, tokens_hist=model_config.tokens_per_history_traj,
                   tokens_fut=model_config.tokens_per_future_traj, generation_mode=generation_mode, generate_cot=generate_cot)


def collate_fn(data, model_config=None, padding_side="left"):
    """hydra target for data.collate_fn: the recipe's collate (left padding, label mask from the special tokens)."""
    return get_processor(model_config).collate_fn(data, padding_side=padding_side)
