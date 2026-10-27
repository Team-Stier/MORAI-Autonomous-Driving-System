"""--lora-watch with LLM + expert LoRAs (DAgger llm+expert) on the real delivery and a GPU (5090 / 5080), on live
requests (model_server --dump), through the actor's path (EngineBackend run_batch, `sims` rows, one expert set):
  swap      seconds per new latest.json: llm+expert (v1), expert only (v2: the LLM back to the delivery), v1 again (v3),
            zero B (v4 = the delivery);
  bits      v2 leaves every LLM byte as delivered, v3 reproduces v1's bytes and actions bit for bit, v4 the delivery's;
  fresh     sampled LLM and expert layers (NVFP4 / FP8 / BF16) equal a fresh quantization of W + s B A (manifest amax);
  timing    model_ms per batch on the same requests before and after the swap (budget 100 ms with sims 1);
  bf16      the engine's actions without / with v1 vs the delivery's BF16 weights without / with v1 merged in BF16
            (FastEngine, eager): the LoRA'd gap should stay the size of the plain quantization gap.
LoRA: a learner file (train/dagger publish) if given, else random r 16 on every LLM + expert decoder linear.

  (cwd src/e2e_driver/server; the pod: V=/workspace)
  HF_HUB_OFFLINE=1 HF_HOME=$V/models/hf MORAI_VLA_PYLIB=$V/morai-student/pylib $V/venv-engine/bin/python \
      ../test/lora_llm_check.py <deliver dir> <dir of req_*.bin> [sims=1] [learner LoRA .safetensors] [out.json]
"""
import gc
import json
import math
import os
import shutil
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "server"), str(HERE.parent / "src"), str(HERE.parent / "server" / "vendor")]
sys.path += [os.path.expanduser(p) for p in [os.environ.get("MORAI_VLA_PYLIB", "")] if p]

import numpy as np  # noqa: E402
import torch  # noqa: E402
from safetensors.torch import save_file  # noqa: E402

import engine  # noqa: E402
from assemble import build_sample  # noqa: E402
from backends import EngineBackend  # noqa: E402
from e2e_driver import wire  # noqa: E402
from fast_engine import FastEngine  # noqa: E402
from nvfp4 import F4Linear, F8Linear  # noqa: E402

R = {}
TARGET = ("expert.layers.", engine.LLM)


def report(k, v):
    R[k] = v
    print(k, json.dumps(v, default=str), flush=True)


def served(mod):
    return [mod.wq.view(torch.uint8), mod.ws.view(torch.uint8)] if isinstance(mod, F4Linear) else \
        [mod.w8.view(torch.uint8)] if isinstance(mod, F8Linear) else [mod.weight]


def main():
    deliver, reqdir = sys.argv[1:3]
    S = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    given = sys.argv[4] if len(sys.argv) > 4 and sys.argv[4] not in ("", "-") else None
    out_json = sys.argv[5] if len(sys.argv) > 5 else None
    reqs = [wire.unpack(f.read_bytes()) for f in sorted(Path(reqdir).glob("req_*.bin"))]
    watch = Path(tempfile.mkdtemp()) / "latest.json"
    be = EngineBackend(deliver, seed=0, sims=S, lora_watch=str(watch))
    fe, model = be.fast, be.model
    man = json.load(open(Path(deliver) / "quant_manifest.json"))["layers"]
    names = [n for n in be.base if n.startswith(TARGET)]
    report("load", dict(s=round(be.load_s, 1), info=model.engine_info, requests=len(reqs), sims=S,
                        base_gb=round(sum(t.numel() * 2 for t in be.base.values()) / 2**30, 2),
                        base_pinned=next(iter(be.base.values())).is_pinned(), llm_layers=sum(n.startswith(engine.LLM) for n in names),
                        expert_layers=sum(n.startswith("expert.") for n in names), max_gb=round(torch.cuda.max_memory_allocated() / 2**30, 2)))

    prep = lambda i: be.prepare(build_sample(reqs[i % len(reqs)], key=f"check/{i}"))

    def inputs(p):
        (b, keys, lay), _ = p
        b = be._dev(b)
        return dict(td=b["tokenized_data"], hx=b["ego_history_xyz"], hr=b["ego_history_rot"], keys=keys, lay=lay)

    x0 = lambda i: torch.randn(1, 64, 2, device="cuda", generator=torch.Generator("cuda").manual_seed(1000 + i))
    zeros = torch.zeros(10, 1, 64, 2, device="cuda")
    n_out = -(-8 // S) * S
    idx = list(range(300, 300 + n_out))                  # the output-check requests (kept prepared)
    P = {i: prep(i) for i in idx}

    def outs(f):
        """deterministic normalised actions (n_out, 64, 2) for idx, fixed x0, every image new, S rows a batch."""
        acts = []
        for c in range(0, n_out, S):
            f.caches = {}
            a, _, _ = f.run_batch([dict(inputs(P[i]), sim=k, x0=x0(i), noise=zeros) for k, i in enumerate(idx[c:c + S])])
            acts.append(a.cpu())
        return torch.cat(acts)

    def timing():
        """model_ms of T consecutive batches (one stretch of the drive per simulator), first two dropped (all new)."""
        fe.caches, ms = {}, []
        for j in range(32):
            ms.append(be.run_batch([(k, {}, prep(200 * k + j), 0.0) for k in range(S)])[0]["timing"]["model_ms"])
        ms = ms[2:]
        return dict(p50=round(float(np.median(ms)), 1), p95=round(float(np.percentile(ms, 95)), 1), max=max(ms),
                    over_100ms=round(float(np.mean(np.array(ms) > 100)), 3))

    def snap():
        return {n: [t.clone() for t in served(model.get_submodule(n))] for n in names}

    def same(a, b, keep=lambda n: True):
        return all(all(torch.equal(x, y) for x, y in zip(a[n], b[n])) for n in a if keep(n))

    def publish(version, t, scale):
        name = f"v{version:05d}.safetensors"
        save_file(t, str(watch.parent / name), metadata={"scale": str(scale), "version": str(version)})
        tmp = watch.with_suffix(".tmp")
        tmp.write_text(json.dumps({"version": version, "path": name})); os.replace(tmp, watch)
        took = be.poll_lora()
        assert took is not None and be.lora_version == version, (took, be.lora_version)
        return round(took, 2)

    if given:
        lora1, scale, _ = engine.read_lora(given)
    else:
        g, scale, lora1 = torch.Generator().manual_seed(1), 2.0, {}
        for n in names:
            o, i_ = be.base[n].shape
            lora1[n] = ((torch.randn(16, i_, generator=g) / math.sqrt(i_)).to(torch.bfloat16),
                        (torch.randn(o, 16, generator=g) * 0.01).to(torch.bfloat16))
    flat = lambda lo, zero=False: {k: v for n, (A, B) in lo.items()
                                   for k, v in ((n + ".lora_A", A), (n + ".lora_B", torch.zeros_like(B) if zero else B))}
    report("lora", dict(file=given, scale=scale, llm=sum(n.startswith(engine.LLM) for n in lora1),
                        expert=sum(n.startswith("expert.") for n in lora1)))

    for _ in range(2):                                    # compile + capture (minutes the first time)
        outs(fe)
    e0, d0, t0 = outs(fe), snap(), timing()

    swap = [publish(1, flat(lora1), scale)]
    e1, d1, t1 = outs(fe), snap(), timing()
    kind = lambda n: type(model.get_submodule(n)).__name__
    fresh = []
    for pre in TARGET:                                    # merged = load() of a merged delivery (F4Linear / F8Linear init)
        for k in ("F4Linear", "F8Linear", "Linear"):
            for n in [x for x in lora1 if x.startswith(pre) and kind(x) == k][::40][:3]:
                A, B = lora1[n]
                w = (be.base[n].cuda().float() + scale * (B.cuda().float() @ A.cuda().float())).to(torch.bfloat16)
                lin, q = SimpleNamespace(weight=SimpleNamespace(data=w), bias=None), man.get(n)
                ref = served(engine.FORMATS[q["format"]](lin, q["weight_amax"], q["input_amax"])) if q else [w]
                fresh.append((n, k, all(torch.equal(x, y) for x, y in zip(ref, served(model.get_submodule(n))))))

    expert_only = {n: v for n, v in lora1.items() if n.startswith("expert.")}
    swap.append(publish(2, flat(expert_only), scale))
    llm_back = same(snap(), d0, lambda n: n.startswith(engine.LLM))
    swap.append(publish(3, flat(lora1), scale))
    e3, v3_bytes = outs(fe), same(snap(), d1)
    swap.append(publish(4, flat(lora1, zero=True), scale))
    e4, v4_bytes = outs(fe), same(snap(), d0)
    report("D_swap", dict(swap_s=swap, versions="v1 llm+expert, v2 expert only, v3 = v1, v4 zero B",
                          lora_changes_output=float((e1 - e0).abs().max()), llm_back_after_expert_only=llm_back,
                          v3_bytes_equal_v1=v3_bytes, v3_actions_equal_v1=torch.equal(e3, e1),
                          v4_bytes_equal_delivery=v4_bytes, v4_actions_equal_delivery=torch.equal(e4, e0),
                          fresh_quant_equal=fresh, model_ms_before=t0, model_ms_after=t1,
                          max_gb=round(torch.cuda.max_memory_allocated() / 2**30, 2)))

    # BF16 reference: the delivery's weights unquantized, + v1 merged in BF16, the same FastEngine computation eagerly
    hist = [(d["hx"].cpu(), d["hr"].cpu()) for d in (inputs(P[i]) for i in idx)]
    sp = be.sp_cpu
    be.model = be.fast = be.base = fe = model = d0 = d1 = None              # be stays for _dev
    torch._dynamo.reset(); gc.collect(); torch.cuda.empty_cache()
    bf = engine.load(deliver, quantize=False)
    rfe = FastEngine(bf, compile=False, use_graphs=False, batch=S)
    r0 = outs(rfe)
    with torch.no_grad():
        for n, (A, B) in lora1.items():
            w = bf.get_submodule(n).weight
            w.copy_((w.float() + scale * (B.cuda().float() @ A.cuda().float())).to(torch.bfloat16))
    r1 = outs(rfe)

    def traj(a):
        return torch.stack([sp.action_to_traj(a[j:j + 1], hx[:, 0], hr[:, 0])[0][0, :, :2].float() for j, (hx, hr) in enumerate(hist)])
    ade = lambda a, b: round(float((traj(a) - traj(b)).norm(dim=-1).mean()), 4)
    rel = lambda a, b: round(float((a - b).norm() / b.norm()), 4)
    report("bf16_reference", dict(n=n_out, quant_gap_no_lora=dict(max=float((e0 - r0).abs().max()), ade_m=ade(e0, r0)),
                                  quant_gap_lora=dict(max=float((e1 - r1).abs().max()), ade_m=ade(e1, r1)),
                                  lora_effect_ade_m=dict(engine=ade(e1, e0), bf16=ade(r1, r0)),
                                  lora_effect_rel_err=rel(e1 - e0, r1 - r0),
                                  lora_effect_cos=round(float(torch.nn.functional.cosine_similarity((e1 - e0).flatten(), (r1 - r0).flatten(), 0)), 4)))
    shutil.rmtree(watch.parent, ignore_errors=True)
    if out_json:
        json.dump(R, open(out_json, "w"), indent=1, default=str)
    print("done", flush=True)


if __name__ == "__main__":
    main()
