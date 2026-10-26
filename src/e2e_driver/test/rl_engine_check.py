"""RL actor engine checks on the 5080 (RL_PLAN 3.1, 5 A; model_server --sims / --expert2 / --lora-watch), on
consecutive live requests (model_server --dump):
  A  the deployment path (FastEngine.run, deterministic Euler) is bit for bit the one before the RL changes
     (BASE_FAST_ENGINE = fast_engine.py of eval 1b69cef), and as fast;
  F  the same without Inductor (compile=False: eager ops in CUDA graphs), i.e. the refactor alone;
  B  RL sampler, one simulator: sigma 0 equals the single path; with sigma the chain and logp agree with an eager
     recomputation of v(x_k, t_k) on the same prefix (what the learner does); step time vs the deterministic path;
  C  three simulators in one batch: decisions/s vs one at a time (one ViT cache per simulator either way), each row vs
     the single path with the same noise, and the CPU preparation time per request;
  D  LoRA: a new file in the watch path merges into the second expert set in place (seconds); the base set stays bit
     for bit; merged layers equal a fresh quantization of W + s B A (load() of a merged delivery); another LoRA and
     back reproduces the first bit for bit (graphs follow the swapped weights); switching sets per step costs nothing;
  E  a batch with both expert sets (2 + 1 rows).

  winpc (cwd src/e2e_driver/server): HF_HUB_OFFLINE=1 MORAI_VLA_PYLIB=~/morai-student/pylib BASE_FAST_ENGINE=<old file> \
      ~/morai-eval/venv-engine/bin/python ../test/rl_engine_check.py <deliver dir> <dir of req_*.bin> [stages ABCDEF] [out.json]
"""
import importlib.util
import json
import math
import os
import sys
import tempfile
import time
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
from backends import EngineBackend, rl_logp  # noqa: E402
from e2e_driver import wire  # noqa: E402
from nvfp4 import F4Linear, F8Linear  # noqa: E402

SIGMA, K = 0.1, 10
R = {}


def report(k, v):
    R[k] = v
    print(k, json.dumps(v, default=str), flush=True)


med = lambda xs: round(float(np.median(xs)), 2)
gb = lambda: round(torch.cuda.max_memory_allocated() / 2**30, 2)


def main():
    deliver, reqdir = sys.argv[1:3]
    stages = sys.argv[3] if len(sys.argv) > 3 else "ABCDE"
    out_json = sys.argv[4] if len(sys.argv) > 4 else None
    reqs = [wire.unpack(f.read_bytes()) for f in sorted(Path(reqdir).glob("req_*.bin"))]
    watch = Path(tempfile.mkdtemp()) / "latest.json"                      # the learner's <out>/lora/latest.json
    be = EngineBackend(deliver, seed=0, sims=3, expert2="", lora_watch=str(watch))
    fe = be.fast
    report("load", dict(s=round(be.load_s, 1), info=be.model.engine_info, max_gb=gb(), requests=len(reqs)))
    man = json.load(open(Path(deliver) / "quant_manifest.json"))["layers"]

    def prep(i):
        s = build_sample(reqs[i % len(reqs)], key=f"check/{i}")
        t = time.perf_counter()
        p = be.prepare(s)
        return p, time.perf_counter() - t

    def inputs(p):                                       # fresh device copies per engine call
        (b, keys, lay), _ = p
        b = be._dev(b)
        return dict(td=b["tokenized_data"], hx=b["ego_history_xyz"], hr=b["ego_history_rot"], keys=keys, lay=lay)

    def x0(i):
        return torch.randn(1, 64, 2, device="cuda", generator=torch.Generator("cuda").manual_seed(1000 + i))

    def eps(i):
        return torch.randn(K, 1, 64, 2, device="cuda", generator=torch.Generator("cuda").manual_seed(5000 + i))

    def single(p, i, e=0, cache=None):
        if cache is not None:
            fe.vit_cache = cache
        d = inputs(p)
        torch.cuda.synchronize(); t = time.perf_counter()
        a, st = fe.run(d["td"], d["hx"], d["hr"], x0(i), d["keys"], d["lay"], expert=e)
        return a, dict(st), time.perf_counter() - t

    def batch(ps, idx, sims, noise, e=0):
        items = [dict(inputs(p), sim=k, x0=x0(i), noise=n) for p, i, k, n in zip(ps, idx, sims, noise)]
        torch.cuda.synchronize(); t = time.perf_counter()
        a, ch, st = fe.run_batch(items, expert=e)
        return a, ch, st, time.perf_counter() - t

    zeros = torch.zeros(K, 1, 64, 2, device="cuda")
    nxt = 0                                              # next request index (stages use their own stretches)

    spec = importlib.util.spec_from_file_location("fast_engine_base", os.environ["BASE_FAST_ENGINE"])
    old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)

    if "F" in stages:                                    # the refactor alone: eager ops in CUDA graphs, no Inductor
        from fast_engine import FastEngine
        ofe, nfe = old.FastEngine(be.model, compile=False), FastEngine(be.model, compile=False)
        rows = []
        for i in range(10):
            p, _ = prep(500 + i)
            d1, d2 = inputs(p), inputs(p)
            a1, _ = ofe.run(d1["td"], d1["hx"], d1["hr"], x0(i), d1["keys"], d1["lay"])
            a2, _ = nfe.run(d2["td"], d2["hx"], d2["hr"], x0(i), d2["keys"], d2["lay"])
            rows.append((torch.equal(a1, a2), (a1 - a2).abs().max().item()))
            print("F", i, rows[-1], flush=True)
        report("F_refactor_eager_bit_identical", dict(all_equal=all(r[0] for r in rows), max_abs=max(r[1] for r in rows), n=len(rows)))
        del ofe, nfe

    if "A" in stages:
        ofe = old.FastEngine(be.model)
        rows = []
        for i in range(14):
            p, _ = prep(i)
            d = inputs(p)
            torch.cuda.synchronize(); t = time.perf_counter()
            a_old, st_old = ofe.run(d["td"], d["hx"], d["hr"], x0(i), d["keys"], d["lay"])
            t_old = time.perf_counter() - t
            a_new, st_new, t_new = single(p, i)
            rows.append((torch.equal(a_old, a_new), (a_old - a_new).abs().max().item(), t_old, t_new, st_new["new_images"]))
            print("A", i, rows[-1], flush=True)
        report("A_deploy_bit_identical", dict(all_equal=all(r[0] for r in rows), max_abs=max(r[1] for r in rows), n=len(rows),
                                              old_ms=med([1e3 * r[2] for r in rows[2:]]), new_ms=med([1e3 * r[3] for r in rows[2:]])))
        del ofe
        nxt = 14

    if "B" in stages:
        rows, lat = [], dict(single=[], det=[], noisy=[])
        fe.vit_cache, fe.caches = {}, {}
        for j in range(12):
            i = nxt + j
            p, _ = prep(i)
            a1, _, t1 = single(p, i)
            ad, chd, _, td_ = batch([p], [i], [0], [zeros])                      # cache sim 0
            e_ = eps(i); sig = torch.full((K,), SIGMA)
            an, chn, _, tn = batch([p], [i], [1], [SIGMA * e_])                  # cache sim 1 (same images, same state)
            row = dict(det_vs_single=(ad - a1).abs().max().item(), det_equal=torch.equal(ad, a1),
                       chain_x0=torch.equal(chn[0, 0], x0(i)[0]))
            if j in (6, 11):                                  # the learner's recomputation: v at each (x_k, t_k), eager
                d = inputs(p)
                td = {k: (v.cuda() if torch.is_tensor(v) else v) for k, v in d["td"].items()}
                ts = torch.linspace(0, 1, K + 1, device="cuda")
                vs = []
                for k in range(K):
                    _, _, v = be.model.sample_trajectories(dict(td), d["hx"], d["hr"], num_samples=1, num_steps=K,
                                                           velocity_at=(chn[k, 0:1], ts[k:k + 1]))
                    vs.append(v[0, 0])
                mu = chn[:K, 0] + (ts[1:] - ts[:-1]).view(K, 1, 1) * torch.stack(vs)
                implied = (chn[1:, 0] - mu) / SIGMA
                lp_eager = rl_logp(implied[:, None].cpu(), sig)
                lp = rl_logp(e_.cpu(), sig)
                row.update(logp_sum=float(lp.sum()), logp_sum_eager=float(lp_eager.sum()),
                           logp_step_max_abs=float((lp - lp_eager).abs().max()), eps_rms_err=float((implied - e_[:, 0]).pow(2).mean().sqrt()))
            lat["single"].append(t1); lat["det"].append(td_); lat["noisy"].append(tn)
            rows.append(row)
            print("B", j, row, flush=True)
        report("B_rl_sampler", dict(det_equal_single=all(r["det_equal"] for r in rows), det_max_abs=max(r["det_vs_single"] for r in rows),
                                    chain_starts_at_x0=all(r["chain_x0"] for r in rows),
                                    eager=[{k: r[k] for k in r if k.startswith(("logp", "eps"))} for r in rows if "logp_sum" in r],
                                    ms_single=med([1e3 * x for x in lat["single"][2:]]), ms_batch1_det=med([1e3 * x for x in lat["det"][2:]]),
                                    ms_batch1_noisy=med([1e3 * x for x in lat["noisy"][2:]]), sigma=SIGMA))
        nxt += 12

    if "C" in stages:
        offs = [nxt, nxt + 80, nxt + 160]                     # three stretches of the drive = three simulators
        scache, rows, prep_s = [{}, {}, {}], [], []
        fe.caches = {}
        for j in range(26):
            ps, idx = [], []
            for k in range(3):
                p, s_ = prep(offs[k] + j); ps.append(p); idx.append(offs[k] + j); prep_s.append(s_)
            noisy = j >= 20                                     # the last batches with exploration noise (timing)
            a, ch, st, tb = batch(ps, idx, [0, 1, 2], [SIGMA * eps(i) if noisy else zeros for i in idx])
            ts_, diffs = [], []
            for k in range(3):
                ak, _, tk = single(ps[k], idx[k], cache=scache[k])
                ts_.append(tk)
                if not noisy:
                    diffs.append((a[k:k + 1] - ak).abs().max().item())
            rows.append(dict(batch_s=tb, single_s=sum(ts_), diff=max(diffs) if diffs else None, noisy=noisy, new=st["new_images"],
                             prefill_ms=st["prefill_ms"], expert_ms=st["expert_ms"]))
            print("C", j, rows[-1], flush=True)
        det, noi = [r for r in rows[2:] if not r["noisy"]], [r for r in rows if r["noisy"]]
        report("C_batch3", dict(decisions_per_s_batch=round(3 / np.median([r["batch_s"] for r in det]), 2),
                                decisions_per_s_single=round(3 / np.median([r["single_s"] for r in det]), 2),
                                decisions_per_s_batch_noisy=round(3 / np.median([r["batch_s"] for r in noi]), 2),
                                batch_ms=med([1e3 * r["batch_s"] for r in det]), single3_ms=med([1e3 * r["single_s"] for r in det]),
                                prefill_ms=med([r["prefill_ms"] for r in det]), expert_ms=med([r["expert_ms"] for r in det]),
                                row_vs_single_max_abs=max(r["diff"] for r in det), prep_ms_per_request=med([1e3 * x for x in prep_s[6:]]),
                                max_gb=gb()))
        nxt += 200

    if "D" in stages:
        names = [n for n, m in fe.parts[1].named_modules() if n.startswith("expert.layers.") and isinstance(m, (F4Linear, F8Linear, torch.nn.Linear))]

        def write_lora(seed, version):
            g, t = torch.Generator().manual_seed(seed), {}
            for n in names:
                o, i_ = be.base[n].shape
                t[n + ".lora_A"] = (torch.randn(16, i_, generator=g) / math.sqrt(i_)).to(torch.bfloat16)
                t[n + ".lora_B"] = (torch.randn(o, 16, generator=g) * 0.02).to(torch.bfloat16)
            name = f"v{version:05d}.safetensors"                 # as the learner's publish(): the file, then latest.json
            save_file(t, str(watch.parent / name), metadata={"scale": "2.0", "rank": "16", "version": str(version)})
            tmp = watch.with_suffix(".tmp")
            tmp.write_text(json.dumps({"version": version, "path": name, "scale": 2.0, "rank": 16})); os.replace(tmp, watch)

        p, _ = prep(nxt)
        run = lambda e: single(p, 777, e, cache={})[0]         # every image new each time: identical inputs
        a0, a1_copy = run(0), run(1)
        write_lora(1, 1)
        took1 = be.poll_lora()
        a1, a0_after = run(1), run(0)
        lora, scale, version = engine.read_lora(watch.parent / "v00001.safetensors")
        fresh_equal = []
        for n in names[::max(1, len(names) // 6)]:            # merged = load() of a merged delivery (F4Linear/F8Linear init)
            A, B = lora[n]
            w = (be.base[n].cuda().float() + scale * (B.cuda().float() @ A.cuda().float())).to(torch.bfloat16)
            mod, q = fe.parts[1].get_submodule(n), man.get(n)
            lin = SimpleNamespace(weight=SimpleNamespace(data=w), bias=None)
            if isinstance(mod, F4Linear):
                f = F4Linear(lin, q["weight_amax"], q["input_amax"])
                ok = torch.equal(f.wq.view(torch.uint8), mod.wq.view(torch.uint8)) and torch.equal(f.ws.view(torch.uint8), mod.ws.view(torch.uint8))
            elif isinstance(mod, F8Linear):
                ok = torch.equal(F8Linear(lin, q["weight_amax"], q["input_amax"]).w8.view(torch.uint8), mod.w8.view(torch.uint8))
            else:
                ok = torch.equal(w, mod.weight)
            fresh_equal.append((n, type(mod).__name__, ok))
        write_lora(2, 2); took2 = be.poll_lora(); a2 = run(1)
        write_lora(1, 3); took3 = be.poll_lora(); a1b = run(1)
        sw = dict(alt=[], const=[])
        fe.vit_cache = {}
        for j in range(24):                                   # per-step switching vs one set, consecutive requests
            pj, _ = prep(nxt + 1 + j)
            _, st, _ = single(pj, nxt + 1 + j, e=j % 2 if j < 12 else 0)
            (sw["alt"] if j < 12 else sw["const"]).append(st["llm_expert_ms"])
        report("D_lora", dict(swap_s=[round(x, 2) for x in (took1, took2, took3)], version=be.lora_version,
                              copy_equals_base=torch.equal(a1_copy, a0), base_unchanged=torch.equal(a0_after, a0),
                              lora_changes_output=(a1 - a0).abs().max().item(), second_lora_differs=(a2 - a1).abs().max().item(),
                              back_to_first_equal=torch.equal(a1b, a1), fresh_quant_equal=fresh_equal,
                              llm_expert_ms_alternating=med(sw["alt"][2:]), llm_expert_ms_one_set=med(sw["const"][2:]), max_gb=gb()))
        nxt += 30

    if "E" in stages:                                         # 2 rows on the base set + 1 on the second, as run_batch groups
        rows = []
        fe.caches = {}
        for j in range(8):
            jobs = []
            for k in range(3):
                p, _ = prep(nxt + 40 * k + j)
                jobs.append((k, {"expert": 1 if k == 1 else 0}, p, SIGMA))
            torch.cuda.synchronize(); t = time.perf_counter()
            outs = be.run_batch(jobs)
            rows.append(time.perf_counter() - t)
            print("E", j, round(1e3 * rows[-1], 1), [o["expert"] for o in outs], [o["timing"].get("group") for o in outs], flush=True)
        report("E_mixed_batch", dict(ms=med([1e3 * x for x in rows[2:]]), experts=[o["expert"] for o in outs],
                                     logp_sum=[round(o["logp_sum"], 1) for o in outs], max_gb=gb()))

    if out_json:
        json.dump(R, open(out_json, "w"), indent=1, default=str)
    print("done", flush=True)


if __name__ == "__main__":
    main()
