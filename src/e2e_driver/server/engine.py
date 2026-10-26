"""The RTX 5080 engine of a delivered student (train/scripts/qad_export.py deliver: final/ + quant_manifest.json).

load(deliver_dir): the student's own architecture (StudentVLA from final/config.json) with its BF16 weights after QAD,
then every linear listed in the manifest becomes a real NVFP4 or FP8 GEMM (nvfp4.F4Linear / F8Linear) with the scales
the QAD quantizers trained with; layers not listed stay BF16 (LiDAR ViT, mergers, embeddings / LM head, action in/out
proj), as in the fake-quantized QAD student. ModelOpt is not needed: final/'s quantizer buffers are ignored (only
'*_quantizer.*' keys may be left over) and modelopt_state.pth is not read.
A new delivery swaps in by pointing E2E_MODEL_CKPT (runtime.env) at its directory.
"""
import json
import os
import tempfile
from pathlib import Path

import torch

from nvfp4 import F4Linear, F8Linear

FORMATS = {"nvfp4": F4Linear, "fp8": F8Linear}


def _plain_dir(final):
    """final/ without modelopt_state.pth (its presence makes StudentVLA.build restore the ModelOpt quantizers)."""
    d = Path(tempfile.mkdtemp(prefix="engine_"))
    for f in Path(final).iterdir():
        if f.name != "modelopt_state.pth":
            os.symlink(f.resolve(), d / f.name)
    return d


def load(deliver, vlm="Qwen/Qwen3-VL-4B-Instruct", device="cuda:0", attn="sdpa", quantize=True):
    from safetensors import safe_open
    from morai_vla.student import StudentVLA
    deliver = Path(deliver)
    final = deliver / "final"
    model = StudentVLA.build(init=str(_plain_dir(final)), vlm_name_or_path=vlm, attn_implementation=attn, dtype="bfloat16",
                             device_map={"": device}, strict=False).eval()
    with safe_open(final / "model.safetensors", "pt") as f:
        saved = set(f.keys())
    sd = model.state_dict()
    ptrs = {sd[k].data_ptr() for k in saved & set(sd)}
    missing = sorted(k for k in set(sd) - saved
                     if not k.endswith(".freqs") and sd[k].data_ptr() not in ptrs)   # Fourier buffers come from config; tied LM head
    extra = sorted(k for k in saved - set(sd) if "_quantizer." not in k)
    assert not extra and not missing, (missing[:10], extra[:10])
    man = json.load(open(deliver / "quant_manifest.json"))["layers"]
    counts = {}
    if quantize:
        for name, q in man.items():
            parent, _, child = name.rpartition(".")
            lin = model.get_submodule(name)
            assert isinstance(lin, torch.nn.Linear) and list(lin.weight.shape) == q["shape"], (name, type(lin))
            setattr(model.get_submodule(parent), child, FORMATS[q["format"]](lin, q["weight_amax"], q["input_amax"]))
            counts[q["format"]] = counts.get(q["format"], 0) + 1
        torch.cuda.empty_cache()
    model.engine_info = dict(deliver=str(deliver), layers=counts, bf16_left_over=len(man) - sum(counts.values()))
    return model
