"""The RTX 5080 engine of a delivered student (train/scripts/qad_export.py deliver: final/ + quant_manifest.json).

load(deliver_dir): the student's own architecture (StudentVLA from final/config.json, built empty), then the tensors of
final/model.safetensors one at a time straight onto the GPU: every linear listed in the manifest becomes a real NVFP4 or
FP8 GEMM (nvfp4.F4Linear / F8Linear) with the scales the QAD quantizers trained with, and its BF16 weight is dropped at
once; everything else stays BF16 (LiDAR ViT, mergers, embeddings / LM head, action in/out proj), as in the
fake-quantized QAD student. Peak memory is the engine itself (no full BF16 copy in RAM or on the GPU). ModelOpt is not
needed: final/'s quantizer buffers ('*_quantizer.*') are skipped and modelopt_state.pth is not read.
A new delivery swaps in by pointing E2E_MODEL_CKPT (runtime.env) at its directory.
"""
import json
from pathlib import Path
from types import SimpleNamespace

import torch

from nvfp4 import F4Linear, F8Linear

FORMATS = {"nvfp4": F4Linear, "fp8": F8Linear}


def load(deliver, vlm="Qwen/Qwen3-VL-4B-Instruct", device="cuda:0", attn="sdpa", quantize=True):
    from accelerate import init_empty_weights
    from safetensors import safe_open
    from morai_vla.model import exact_action_space, load_config
    from morai_vla.student import StudentVLA
    deliver = Path(deliver)
    final = deliver / "final"
    config = load_config(final, vlm, attn)
    prev = torch.get_default_dtype()
    torch.set_default_dtype(torch.bfloat16)        # as from_pretrained(dtype=bf16): config-derived buffers (Fourier freqs) in bf16
    try:
        with init_empty_weights():                 # parameters on meta, buffers real
            model = StudentVLA(config)
    finally:
        torch.set_default_dtype(prev)
    man = json.load(open(deliver / "quant_manifest.json"))["layers"] if quantize else {}
    counts, rest = {}, {}
    index = final / "model.safetensors.index.json"                     # save_pretrained shards a full-size student (3 files)
    shard_of = json.load(open(index))["weight_map"] if index.exists() else None
    files = sorted(set(shard_of.values())) if shard_of else ["model.safetensors"]
    handles = {n: safe_open(final / n, "pt", device=str(device)) for n in files}
    if shard_of is None:
        shard_of = {k: files[0] for k in handles[files[0]].keys()}
    f = SimpleNamespace(get_tensor=lambda k: handles[shard_of[k]].get_tensor(k))
    saved = set(shard_of)
    for name, q in man.items():
        parent, _, child = name.rpartition(".")
        lin = model.get_submodule(name)
        assert isinstance(lin, torch.nn.Linear) and list(lin.weight.shape) == q["shape"], (name, type(lin))
        bf = lambda k: f.get_tensor(k).to(torch.bfloat16)                      # the model runs (and QAD ran) in BF16
        w = SimpleNamespace(weight=SimpleNamespace(data=bf(name + ".weight")),
                            bias=SimpleNamespace(data=bf(name + ".bias")) if lin.bias is not None else None)
        setattr(model.get_submodule(parent), child, FORMATS[q["format"]](w, q["weight_amax"], q["input_amax"]))
        counts[q["format"]] = counts.get(q["format"], 0) + 1
    own = model.state_dict()
    for k in sorted(saved):
        if k in own:                                                           # cast as from_pretrained(dtype=bf16)
            t = f.get_tensor(k)
            rest[k] = t.to(own[k].dtype) if t.is_floating_point() else t
        else:
            assert "_quantizer." in k or k.rsplit(".", 1)[0] in man, k           # ModelOpt buffers, quantized layers
    model.load_state_dict(rest, strict=False, assign=True)
    model.tie_weights()
    meta = [n for n, p in list(model.named_parameters()) + list(model.named_buffers()) if p.is_meta]
    assert not meta, meta[:10]
    model.to(device).eval()
    exact_action_space(model)
    torch.cuda.empty_cache()
    model.engine_info = dict(deliver=str(deliver), layers=counts, gpu_gb=round(torch.cuda.memory_allocated(device) / 2**30, 2))
    return model
