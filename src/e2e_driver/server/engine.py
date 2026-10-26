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
BITS = {"nvfp4": 4.5, "fp8": 8.0, "bf16": 16.0}                  # per weight with scales (morai_vla.qad.effective_bits)
SEARCHED = ("language_model.layers.", "expert.layers.")           # the QAD budget: LLM decoder + expert linears


class DeliveryError(ValueError):
    """the delivery is not what train/scripts/qad_export.py deliver writes (final/ + quant_manifest.json)."""


def manifest_layers(deliver):
    """quant_manifest.json layers, checked: files present, every layer nvfp4 (block 16) or fp8 with both amax."""
    final = deliver / "final"
    need = [final / "config.json", deliver / "quant_manifest.json"]
    missing = [str(p) for p in need if not p.exists()]
    if not (final / "model.safetensors").exists() and not (final / "model.safetensors.index.json").exists():
        missing.append(str(final / "model.safetensors[.index.json]"))
    if missing:
        raise DeliveryError("missing %s (a delivery is qad_export.py deliver's final/ + quant_manifest.json)" % ", ".join(missing))
    man = json.load(open(deliver / "quant_manifest.json")).get("layers")
    if not man:
        raise DeliveryError("%s has no 'layers'" % (deliver / "quant_manifest.json"))
    bad = {}
    for name, q in man.items():
        if q.get("format") not in FORMATS:
            bad[name] = "format %r (the engine runs nvfp4 | fp8)" % q.get("format")
        elif q["format"] == "nvfp4" and q.get("block_size") != 16:
            bad[name] = "nvfp4 block_size %r (16)" % q.get("block_size")
        elif not q.get("weight_amax") or not q.get("input_amax"):
            bad[name] = "weight_amax %r / input_amax %r (quantizer off or not calibrated)" % (q.get("weight_amax"), q.get("input_amax"))
    if bad:
        raise DeliveryError("%d manifest layers unusable, e.g. %s" % (len(bad), "; ".join("%s: %s" % kv for kv in list(bad.items())[:3])))
    return man


def load(deliver, vlm="Qwen/Qwen3-VL-4B-Instruct", device="cuda:0", attn="sdpa", quantize=True):
    from accelerate import init_empty_weights
    from safetensors import safe_open
    from morai_vla.model import exact_action_space, load_config
    from morai_vla.student import StudentVLA
    deliver = Path(deliver)
    final = deliver / "final"
    man = manifest_layers(deliver) if quantize else {}
    config = load_config(final, vlm, attn)
    prev = torch.get_default_dtype()
    torch.set_default_dtype(torch.bfloat16)        # as from_pretrained(dtype=bf16): config-derived buffers (Fourier freqs) in bf16
    try:
        with init_empty_weights():                 # parameters on meta, buffers real
            model = StudentVLA(config)
    finally:
        torch.set_default_dtype(prev)
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
        try:
            lin = model.get_submodule(name)
        except AttributeError:
            raise DeliveryError("manifest layer %s is not in the StudentVLA of final/config.json" % name) from None
        if not isinstance(lin, torch.nn.Linear) or list(lin.weight.shape) != q.get("shape"):
            raise DeliveryError("%s: manifest shape %s, model %s %s" % (name, q.get("shape"), type(lin).__name__, list(getattr(lin, "weight", torch.empty(0)).shape)))
        if name + ".weight" not in saved:
            raise DeliveryError("final/ has no %s.weight" % name)
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
        elif not ("_quantizer." in k or k.rsplit(".", 1)[0] in man):         # ModelOpt buffers, quantized layers
            raise DeliveryError("final/ tensor %s is not in the StudentVLA of final/config.json" % k)
    model.load_state_dict(rest, strict=False, assign=True)
    model.tie_weights()
    meta = [n for n, p in list(model.named_parameters()) + list(model.named_buffers()) if p.is_meta]
    if meta:
        raise DeliveryError("final/ lacks %d tensors of the model, e.g. %s" % (len(meta), ", ".join(meta[:3])))
    # the QAD budget as the engine runs it: LLM + expert linears by format (BF16 = a searched linear not in the manifest)
    rows = [(n, "bf16", m.weight.shape) for n, m in model.named_modules() if isinstance(m, torch.nn.Linear) and any(p in n for p in SEARCHED)]
    rows += [(n, q["format"], q["shape"]) for n, q in man.items() if any(p in n for p in SEARCHED)]
    size = lambda s: s[0] * s[1]
    budget = {f: sum(1 for r in rows if r[1] == f) for f in ("nvfp4", "fp8", "bf16")}
    budget["effective_bits"] = round(sum(size(r[2]) * BITS[r[1]] for r in rows) / max(sum(size(r[2]) for r in rows), 1), 3)
    model.to(device).eval()
    exact_action_space(model)
    torch.cuda.empty_cache()
    model.engine_info = dict(deliver=str(deliver), layers=counts, llm_expert=budget, gpu_gb=round(torch.cuda.memory_allocated(device) / 2**30, 2))
    return model
