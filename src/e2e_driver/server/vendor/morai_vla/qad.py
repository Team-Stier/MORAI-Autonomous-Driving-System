"""QAD (MODEL_PLAN 4 step 6, 5.1) with NVIDIA ModelOpt (PyPI nvidia-modelopt): the student is fake-quantized at its
deployment precision, then trained a little more with the same losses (BF16 T* KV / velocity KD + GT). Only the forward
is quantized (straight-through); weights and the backward stay BF16/fp32.
- LLM decoder and expert linears: NVFP4 (16-value blocks, e4m3 block scales, fp32 per-tensor scale) or FP8 (per tensor)
  per layer, chosen by AutoQuantize (gradient sensitivity on the distillation loss) under `effective_bits` (6.5 = the
  recipe's example; MODEL_PLAN 7). BF16 is not a choice for them. AutoQuantize groups q/k/v and gate/up layers.
- camera ViT blocks: FP8. BF16: LiDAR ViT, mergers and DeepStack mergers, embeddings / LM head, action in/out proj, KD
  maps. No KV-cache quantizer (BF16). The Euler integration is outside the model (FP32).
The search runs in one process (every rank must quantize alike) and is saved with the student: modelopt_state.pth next
to the HF weights (mto.enable_huggingface_checkpointing); later loads of that directory restore the quantizers.
"""
import json
from pathlib import Path

import modelopt.torch.opt as mto
import modelopt.torch.quantization as mtq

SEARCHED = ["*vlm.model.language_model.layers.*", "*expert.layers.*"]
CAMERA_VIT = "*vlm.model.visual.blocks.*"
NOT_IN_BUDGET = ["*visual*", "*lm_head*", "*action_in_proj*", "*action_out_proj*", "*vit_kd*"]   # budget = LLM + expert only
FORMATS = {"NVFP4_DEFAULT_CFG": "nvfp4", "FP8_DEFAULT_CFG": "fp8"}
BITS = {"nvfp4": 4.5, "fp8": 8.0}                      # per weight, scales included (ModelOpt's effective_bits)
mto.enable_huggingface_checkpointing()                 # save_pretrained / from_pretrained carry modelopt_state.pth


def _fmt_cfg(name, quantizer):
    return next(e["cfg"] for e in getattr(mtq, name)["quant_cfg"] if e["quantizer_name"] == quantizer)


def fixed_config():
    """everything off except the camera ViT blocks in FP8 (weights and inputs, per tensor)."""
    return {"quant_cfg": [{"quantizer_name": "*", "enable": False},
                          {"quantizer_name": f"{CAMERA_VIT}weight_quantizer", "cfg": _fmt_cfg("FP8_DEFAULT_CFG", "*weight_quantizer")},
                          {"quantizer_name": f"{CAMERA_VIT}input_quantizer", "cfg": _fmt_cfg("FP8_DEFAULT_CFG", "*input_quantizer")}],
            "algorithm": "max"}


def search(model, batches, effective_bits=6.5, num_calib_steps=64, num_score_steps=32):
    """AutoQuantize in place: batches = model-call kwargs (the training collate); the score is the training loss."""
    model, state = mtq.auto_quantize(
        model, constraints={"effective_bits": effective_bits, "cost": {"excluded_module_name_patterns": NOT_IN_BUDGET}},
        fixed_quantization_config=fixed_config(),
        module_search_spaces=[{"module_name_patterns": SEARCHED, "quantization_formats": list(FORMATS), "allow_no_quant": False}],
        data_loader=batches, forward_step=lambda m, b: m(**b), loss_func=lambda out, b: out.loss,
        num_calib_steps=num_calib_steps, num_score_steps=num_score_steps, verbose=True)
    return model, state


def manifest(model):
    """per quantized linear: format, block size, weight / input amax (the per-tensor scales are amax / (6 x 448) for
    NVFP4 and amax / 448 for FP8), weight shape. What the RTX 5080 NVFP4/FP8 engine needs besides the weights."""
    out = {}
    for name, m in model.named_modules():
        wq, iq = getattr(m, "weight_quantizer", None), getattr(m, "input_quantizer", None)
        if wq is None or not wq.is_enabled: continue
        fmt = "nvfp4" if tuple(wq.num_bits) == (2, 1) else "fp8" if tuple(wq.num_bits) == (4, 3) else str(wq.num_bits)
        amax = lambda q: float(q.amax.float().max()) if q is not None and q.is_enabled and q.amax is not None else None
        out[name] = {"format": fmt, "block_size": 16 if fmt == "nvfp4" else None, "weight_amax": amax(wq), "input_amax": amax(iq),
                     "shape": list(m.weight.shape)}
    return out


def effective_bits(man, patterns=("language_model.layers.", "expert.layers.")):
    """weight-weighted bits over the searched layers (NVFP4 4.5, FP8 8)."""
    rows = [(v["shape"][0] * v["shape"][1], BITS.get(v["format"], 16.0)) for k, v in man.items() if any(p in k for p in patterns)]
    n = sum(r[0] for r in rows)
    return round(sum(a * b for a, b in rows) / max(n, 1), 3)


def write_manifest(model, out_dir):
    man = manifest(model)
    count = {f: sum(v["format"] == f for v in man.values()) for f in ("nvfp4", "fp8")}
    meta = {"layers": len(man), "count": count, "effective_bits_searched": effective_bits(man),
            "scales": "NVFP4 per-tensor scale = amax / (6 * 448), block scale e4m3 per 16 values; FP8 scale = amax / 448"}
    json.dump({"meta": meta, "layers": man}, open(Path(out_dir) / "quant_manifest.json", "w"), indent=1)
    return meta
