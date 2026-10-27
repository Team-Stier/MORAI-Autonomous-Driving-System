"""--lora-watch with LLM layers (DAgger llm+expert) without a GPU: a tiny StudentVLA-named model (LLM decoder, ViT,
expert; NVFP4 / FP8 / BF16 linears as engine.load makes them, quantized with the plain-torch NVFP4 quantizer) and a
fake delivery final/. Checked: the base names are the delivery's, a learner-published LoRA merges into LLM + expert
equal to a fresh quantization of W + s B A, the version follows latest.json, wrong names / ViT keys / shapes are
refused with nothing changed, an expert-only LoRA after an LLM one gives the LLM back bit for bit, the same LoRA again
is bit identical, a zero LoRA is the delivery, and with two expert sets an LLM LoRA is refused.

    ~/claude-bench/venv-rl/bin/python test/test_lora_llm.py
"""
import copy
import json
import math
import os
import sys
import tempfile
import types
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "server"), str(HERE.parent / "src"), str(HERE.parent / "server" / "vendor")]

import torch  # noqa: E402
from safetensors.torch import save_file  # noqa: E402
from torch import nn  # noqa: E402

from backends import EngineBackend  # noqa: E402

if "triton" not in sys.modules:                     # no Triton on the Mac: nvfp4 imports, its kernel is never launched
    tl = types.ModuleType("triton.language")
    sys.modules.update(triton=types.SimpleNamespace(jit=lambda f: f, language=tl), **{"triton.language": tl})
import engine  # noqa: E402
import nvfp4  # noqa: E402
from nvfp4 import F4Linear, F8Linear  # noqa: E402

nvfp4.quant_nvfp4 = engine.quant_nvfp4 = nvfp4.quant_nvfp4_torch    # bit-equal to the Triton one (nvfp4._selfcheck)
torch.cuda.synchronize = lambda *a: None
FMT = dict(q_proj="nvfp4", k_proj="nvfp4", gate_proj="nvfp4", down_proj="nvfp4", v_proj="fp8")   # o / up stay BF16


def make(tmp):
    """the tiny model (served form), its manifest and a delivery dir with the BF16 weights in final/."""
    torch.manual_seed(0)
    lin = lambda i, o: nn.Linear(i, o, bias=False, dtype=torch.bfloat16)

    def layer(h=64, f=128):
        L, L.self_attn, L.mlp = nn.Module(), nn.Module(), nn.Module()
        for n in ("q_proj", "k_proj", "v_proj", "o_proj"):
            setattr(L.self_attn, n, lin(h, h))
        L.mlp.gate_proj, L.mlp.up_proj, L.mlp.down_proj = lin(h, f), lin(h, f), lin(f, h)
        return L
    m = nn.Module()
    m.vlm, m.vlm.model = nn.Module(), nn.Module()
    m.vlm.model.language_model = nn.Module()
    m.vlm.model.language_model.layers = nn.ModuleList([layer(), layer()])
    m.vlm.model.visual = nn.Module()
    m.vlm.model.visual.blocks = nn.ModuleList([nn.Module()])
    m.vlm.model.visual.blocks[0].qkv = lin(64, 192)
    m.expert = nn.Module()
    m.expert.layers = nn.ModuleList([layer(), layer()])
    m.action_in_proj, m.action_out_proj = lin(64, 64), lin(64, 2)
    with torch.no_grad():
        for p in m.parameters():
            p.normal_(0, 0.1)
    deliver = Path(tmp) / "deliver"
    (deliver / "final").mkdir(parents=True)
    save_file({n: x.contiguous() for n, x in m.state_dict().items()}, str(deliver / "final" / "model.safetensors"))
    man = {}
    for n, mod in list(m.named_modules()):                # as engine.load: manifest linears -> F4Linear / F8Linear
        fmt = "fp8" if n.endswith(".qkv") else FMT.get(n.rsplit(".", 1)[-1])
        if fmt:
            man[n] = dict(format=fmt, weight_amax=float(mod.weight.detach().abs().max()), input_amax=3.0)
            parent, _, child = n.rpartition(".")
            setattr(m.get_submodule(parent), child, engine.FORMATS[fmt](mod, man[n]["weight_amax"], 3.0))
    return m, man, deliver


def backend(m, deliver, watch, two_sets=False):
    """EngineBackend's LoRA state without engine.load / FastEngine (as its __init__ builds it)."""
    be = object.__new__(EngineBackend)
    parts0 = nn.ModuleDict(dict(expert=m.expert, action_in_proj=m.action_in_proj, action_out_proj=m.action_out_proj))
    be.model, be.fast = m, SimpleNamespace(parts=[parts0], lm=m.vlm.model.language_model)
    be.lora_watch, be.lora_mtime, be.lora_version, be.lora_names = str(watch), None, 0, set()
    be.base = engine.expert_base(deliver, parts0)
    if two_sets:
        be.fast.parts.append(copy.deepcopy(parts0))
    else:
        be.base.update(engine.expert_base(deliver, be.fast.lm.layers, prefix=engine.LLM))
    return be


def state(root):
    """the served bytes of every linear under root."""
    out = {}
    for n, x in root.named_modules():
        if isinstance(x, F4Linear):
            out[n] = (x.wq.view(torch.uint8).clone(), x.ws.view(torch.uint8).clone())
        elif isinstance(x, F8Linear):
            out[n] = (x.w8.view(torch.uint8).clone(),)
        elif isinstance(x, nn.Linear):
            out[n] = (x.weight.detach().clone(),)
    return out


def same(a, b, names=None):
    return all(all(torch.equal(x, y) for x, y in zip(a[n], b[n])) for n in (names or a))


def fresh(man, n, w):
    """the bytes engine.load would serve for weight w (a delivery with W + s B A at n)."""
    q = man.get(n)
    lin = SimpleNamespace(weight=SimpleNamespace(data=w), bias=None)
    if q is None:
        return (w,)
    f = engine.FORMATS[q["format"]](lin, q["weight_amax"], q["input_amax"])
    return (f.wq.view(torch.uint8), f.ws.view(torch.uint8)) if q["format"] == "nvfp4" else (f.w8.view(torch.uint8),)


def publish(watch, version, shapes, seed, zero=False, extra=None):
    """the learner's publish(): vNNNNN.safetensors, then latest.json -> {name: (A, B)}."""
    g, t = torch.Generator().manual_seed(seed), {}
    for n, (o, i) in shapes.items():
        t[n + ".lora_A"] = (torch.randn(4, i, generator=g) / math.sqrt(i)).to(torch.bfloat16)
        t[n + ".lora_B"] = torch.zeros(o, 4, dtype=torch.bfloat16) if zero else (torch.randn(o, 4, generator=g) * 0.05).to(torch.bfloat16)
    t.update(extra or {})
    name = f"v{version:05d}.safetensors"
    save_file(t, str(watch.parent / name), metadata={"scale": "2.0", "rank": "4", "version": str(version)})
    tmp = watch.with_suffix(".tmp")
    tmp.write_text(json.dumps({"version": version, "path": name})); os.replace(tmp, watch)
    return {n: (t[n + ".lora_A"], t[n + ".lora_B"]) for n in shapes}


def merged(be, man, lora):
    return {n: fresh(man, n, (be.base[n].float() + 2.0 * (B.float() @ A.float())).to(torch.bfloat16)) for n, (A, B) in lora.items()}


def test_llm_expert():
    with tempfile.TemporaryDirectory() as tmp:
        m, man, deliver = make(tmp)
        watch = Path(tmp) / "lora" / "latest.json"
        watch.parent.mkdir()
        be = backend(m, deliver, watch)
        llm = [n for n in man if n.startswith(engine.LLM)] + [n for n in state(m) if n.startswith(engine.LLM) and n not in man]
        exp = [n for n in state(m) if n.startswith("expert.layers.")]
        assert set(be.base) == set(llm) | set(exp) | {"action_in_proj", "action_out_proj"}, sorted(be.base)
        assert len(llm) == 14 and not any(n.startswith(engine.VIT) for n in be.base)
        shape = lambda names: {n: tuple(be.base[n].shape) for n in names}
        d0 = state(m)
        assert be.poll_lora() is None                                  # no latest.json yet

        l1 = publish(watch, 1, shape(llm + exp), seed=1)              # llm+expert
        assert be.poll_lora() is not None and be.lora_version == 1
        s1 = state(m)
        assert same(s1, merged(be, man, l1), llm + exp), "merge != fresh quantization of W + s B A"
        assert not any(same(s1, d0, [n]) for n in llm + exp), "a LoRA'd layer kept its delivered bytes"
        assert same(s1, d0, ["action_in_proj", "action_out_proj", "vlm.model.visual.blocks.0.qkv"])
        assert be.poll_lora() is None                                  # latest.json unchanged

        bad = {"unknown": shape(exp[:1]) | {engine.LLM + "9.mlp.up_proj": (128, 64)},     # good layers + one wrong one
               "vit": shape(exp[:1]) | {"vlm.model.visual.blocks.0.qkv": (192, 64)},
               "shape": shape(exp[:1]) | {llm[0]: (64, 128)}}
        for why, shapes in bad.items():
            publish(watch, 2, shapes, seed=2)
            try:
                be.poll_lora()
                raise AssertionError(why + " LoRA accepted")
            except ValueError as e:
                print("refused", why, ":", str(e)[:110])
            assert be.lora_version == 1 and same(state(m), s1), why + ": changed something"
        save_file({"x.weight": torch.zeros(2)}, str(watch.parent / "v00002.safetensors"), metadata={"scale": "1"})
        try:
            be.load_lora(watch.parent / "v00002.safetensors", 2)
            raise AssertionError("non-LoRA keys accepted")
        except ValueError:
            assert be.lora_version == 1 and same(state(m), s1)

        l3 = publish(watch, 3, shape(exp), seed=3)                    # expert only, after an LLM LoRA
        assert be.poll_lora() is not None and be.lora_version == 3
        s3 = state(m)
        assert same(s3, d0, llm), "LLM not back to the delivery after an expert-only LoRA"
        assert same(s3, merged(be, man, l3), exp)
        assert be.lora_names == set(exp)

        publish(watch, 4, shape(llm + exp), seed=1)                   # v1's weights again
        assert be.poll_lora() is not None and same(state(m), s1), "the same LoRA again is not bit identical"
        publish(watch, 5, shape(llm + exp), seed=5, zero=True)        # B = 0: the delivery (learner version 0)
        assert be.poll_lora() is not None and same(state(m), d0), "zero LoRA != delivery"
        print("llm+expert ok:", len(llm), "LLM +", len(exp), "expert layers; versions", 1, 3, 4, 5)


def test_two_sets():
    with tempfile.TemporaryDirectory() as tmp:
        m, man, deliver = make(tmp)
        watch = Path(tmp) / "latest.json"
        be = backend(m, deliver, watch, two_sets=True)
        d0, e0 = state(m), state(be.fast.parts[1])
        assert not any(n.startswith(engine.LLM) for n in be.base)
        llm = [n for n in d0 if n.startswith(engine.LLM)]
        exp = [n for n in e0 if n.startswith("expert.layers.")]
        assert llm[0].endswith("q_proj")                                # (64, 64) in make()
        publish(watch, 1, {llm[0]: (64, 64)} | {n: tuple(be.base[n].shape) for n in exp[:2]}, seed=1)
        try:
            be.poll_lora()
            raise AssertionError("LLM LoRA accepted with two expert sets")
        except ValueError as e:
            print("refused two sets:", str(e)[-70:])
        assert be.lora_version == 0 and same(state(m), d0) and same(state(be.fast.parts[1]), e0)
        l2 = publish(watch, 2, {n: tuple(be.base[n].shape) for n in exp}, seed=2)
        assert be.poll_lora() is not None and be.lora_version == 2
        assert same(state(m), d0), "the delivered set or the LLM changed"
        assert same(state(be.fast.parts[1]), merged(be, man, l2), exp)
        print("two sets ok: LLM LoRA refused, expert LoRA into the second set only")


if __name__ == "__main__":
    test_llm_expert()
    test_two_sets()
