"""Swappable model backends: prepare(sample) -> (batch, timing) on the CPU, run(sample, batch) -> dict(accel (64,),
kappa (64,), v0, xyz (64, 3) or None, timing) on the GPU; the server overlaps the two for consecutive requests.
predict(sample) = run(sample, prepare(sample)). accel/kappa are unnormalised (m/s^2, 1/m): the expert's terminal control
signals.

  hf    a saved student (StudentVLA.build(init=ckpt)) or teacher (TeacherVLA.build(ckpt)) in PyTorch, run with the
        training repo's own inference, model.sample_trajectories (prefix prefill -> <traj_future_start> -> Euler flow
        matching, no CoC). The (a, kappa) it integrates are read off model.diffusion.sample's return value.
  fake  no model: a speed-hold law on the measured history (wiring and latency tests); preprocess=True still builds the
        full model batch with the processor, so its CPU time is in the measurement.
  engine  a delivered QAD student (train/scripts/qad_export.py deliver dir) with real NVFP4/FP8 GEMMs on the 5080
        (engine.py, nvfp4.py); the same inference as hf otherwise. A new delivery = a new --ckpt directory.
        Also the RL actor's engine: run_batch (several simulators, the noisy Euler sampler with log-probs), a second
        expert set picked per request, LoRA updates merged in place (EngineBackend).
"""
import copy
import json
import math
import os
import time

import numpy as np
import torch

from assemble import tokenize


class FakeBackend:
    def __init__(self, v_target=4.0, kappa=0.0, accel_max=1.0, decel_max=3.0, preprocess_config=None, vlm=None):
        self.v_target, self.kappa, self.accel_max, self.decel_max = v_target, kappa, accel_max, decel_max
        self.config = None
        if preprocess_config:
            from morai_vla.model import load_config
            self.config = load_config(preprocess_config, vlm)

    def prepare(self, sample):
        if self.config is None:
            return None, {}
        t = time.perf_counter()
        b = tokenize(sample, self.config)
        return b, dict(preprocess_ms=round(1e3 * (time.perf_counter() - t), 1), prefix_tokens=int(b["tokenized_data"]["input_ids"].shape[1]))

    def run(self, sample, prepared):
        h = sample["ego_history_xyz"][0].numpy().astype(np.float64)                 # (16, 3), t0 last
        v = v0 = float(np.hypot(*(h[-1, :2] - h[-2, :2])) / 0.1)
        accel = np.zeros(64)
        for k in range(64):
            accel[k] = np.clip((self.v_target - v) / 1.0, -self.decel_max, self.accel_max); v += accel[k] * 0.1
        return dict(accel=accel, kappa=np.full(64, sample.get("route_kappa", self.kappa)), v0=v0, xyz=None, timing=dict(prepared[1]))

    def predict(self, sample):
        return self.run(sample, self.prepare(sample))


class HFBackend:
    def __init__(self, ckpt, vlm="Qwen/Qwen3-VL-4B-Instruct", model_class="student", attn="sdpa", dtype="bfloat16",
                 num_steps=10, seed=None, device="cuda:0"):
        from morai_vla.model import TeacherVLA
        from morai_vla.student import StudentVLA
        t = time.perf_counter()
        if model_class == "engine":
            import engine
            self.model = engine.load(ckpt, vlm=vlm, device=device, attn=attn)
        elif model_class == "student":
            self.model = StudentVLA.build(init=ckpt, vlm_name_or_path=vlm, attn_implementation=attn, dtype=dtype, device_map={"": device})
        else:
            self.model = TeacherVLA.build(ckpt, vlm_name_or_path=vlm, attn_implementation=attn, dtype=dtype, device_map={"": device})
        self.model.eval()
        self.device, self.num_steps, self.seed = device, num_steps, seed
        self.load_s = time.perf_counter() - t
        self._last = {}
        orig = self.model.diffusion.sample

        def capture(*a, **k):                     # keep sample_trajectories untouched; remember the (a, kappa) it integrates
            out = orig(*a, **k); self._last["action"] = out; return out

        self.model.diffusion.sample = capture
        self.n = 0

    def prepare(self, sample):
        t = time.perf_counter()
        b = tokenize(sample, self.model.config)
        return b, dict(preprocess_ms=round(1e3 * (time.perf_counter() - t), 1), prefix_tokens=int(b["tokenized_data"]["input_ids"].shape[1]))

    def run(self, sample, prepared):
        m, (b, timing) = self.model, prepared
        t1 = time.perf_counter()
        dev = lambda x: {k: dev(v) if isinstance(v, dict) else v.to(self.device) if torch.is_tensor(v) else v for k, v in x.items()}
        b = dev(b)                                  # host -> device here: the run thread owns the CUDA stream
        if self.seed is not None:
            torch.manual_seed(int(self.seed) + self.n)
        self.n += 1
        hx, hr = b["ego_history_xyz"], b["ego_history_rot"]
        xyz, _ = m.sample_trajectories(b["tokenized_data"], hx, hr, num_samples=1, num_steps=self.num_steps)
        act = self._last.pop("action").float()                                   # (1, 64, 2) normalised
        sp = m.action_space
        accel = act[0, :, 0] * sp.accel_std.float() + sp.accel_mean.float()
        kappa = act[0, :, 1] * sp.curvature_std.float() + sp.curvature_mean.float()
        v0 = sp.estimate_t0_states(hx[:, 0].float(), hr[:, 0].float())["v"]
        torch.cuda.synchronize()
        t2 = time.perf_counter()
        return dict(accel=accel.cpu().numpy().astype(np.float64), kappa=kappa.cpu().numpy().astype(np.float64),
                    v0=float(v0.reshape(-1)[0]), xyz=xyz[0, 0].float().cpu().numpy(),
                    timing=dict(timing, model_ms=round(1e3 * (t2 - t1), 1)))

    def predict(self, sample):
        return self.run(sample, self.prepare(sample))


class EngineBackend(HFBackend):
    """the delivered QAD student on the 5080: engine.load (real NVFP4/FP8 GEMMs) + fast_engine.FastEngine (compiled CUDA
    graphs; camera frames and LiDAR BEVs already seen come from the ViT cache). Compiling and capturing take minutes, so
    __init__ warms up on stored requests (model_server --dump files) before the server listens.

    expert2 (RL_PLAN 5 A): a second expert set in VRAM, the delivery's expert + that LoRA file (engine.read_lora; ''
    = a plain copy), picked per request by sample["expert"] (the node: by route position s). lora_watch: the learner's
    <out>/lora/latest.json {version, path} (Alpamayo-Cosmos train/rl/learner.py publish); poll_lora() merges each new
    version into the last expert set in place (seconds; graphs stay). Version 0 = the delivered expert. With one expert
    set a LoRA may also hold LLM decoder linears (DAgger llm+expert): merged into the LLM the same way (the ViT and its
    cache stay; the prefill reruns every request). sims >= 1: the RL actor (model_server --sims): run_batch over up to `sims` simulators, one ViT cache each,
    one expert graph per batch; warm-up compiles that path instead of the single one."""

    def __init__(self, ckpt, vlm="Qwen/Qwen3-VL-4B-Instruct", num_steps=10, seed=None, device="cuda:0", warmup=(), pad_to=1792,
                 want_xyz=False, sims=0, expert2=None, lora_watch=None):
        import engine
        from fast_engine import FastEngine, fingerprints, layout
        t = time.perf_counter()
        self.model = engine.load(ckpt, vlm=vlm, device=device)
        self.fast, self.fingerprints, self.layout = FastEngine(self.model, num_steps=num_steps, pad_to=pad_to, batch=max(sims, 1)), fingerprints, layout
        self.device, self.num_steps, self.seed, self.n, self.want_xyz = device, num_steps, seed, 0, want_xyz
        self.sp_cpu = copy.deepcopy(self.model.action_space).cpu()          # output conversion off the GPU
        self.lora_watch, self.lora_mtime, self.lora_version, self.base, self.lora_names = lora_watch, None, 0, None, set()
        if expert2 is not None or lora_watch:            # LoRAs merge onto the delivery's BF16 expert (host memory)
            self.base = engine.expert_base(ckpt, self.fast.parts[0])
            if lora_watch and expert2 is None:           # and the LLM's (one expert set; two would share the LLM)
                self.base.update(engine.expert_base(ckpt, self.fast.lm.layers, prefix=engine.LLM))
        if expert2 is not None:
            self.fast.parts.append(copy.deepcopy(self.fast.parts[0]))
            if expert2:
                self.load_lora(expert2)
        if not lora_watch:
            self.base = None
        self.load_s = time.perf_counter() - t
        if warmup:
            from assemble import build_sample
            from e2e_driver import wire
            if not sims:
                for i, f in enumerate(list(warmup) * 3):           # compile + capture, then two timed passes; the first file
                    sample = build_sample(wire.unpack(open(f, "rb").read()), key=f"warmup/{i}")   # of each round has every
                    if i % len(warmup) == 0:                              # image new, the next ones the steady state
                        self.fast.vit_cache.clear()                      # (newest camera frames + BEVs)
                    out = self.run(sample, self.prepare(sample))
                self.warmup_timing = out["timing"]
                for e in range(1, len(self.fast.parts)):              # the other expert set's graph
                    for f in warmup:
                        sample = build_sample(wire.unpack(open(f, "rb").read()), key="warmup/e")
                        self.run(dict(sample, expert=e), self.prepare(sample))
            else:                                                     # every batch size and expert set (graphs, compiles)
                samples = [build_sample(wire.unpack(open(f, "rb").read()), key=f"warmup/{i}") for i, f in enumerate(warmup)]
                prepared = [self.prepare(s) for s in samples]
                for b in range(sims, 0, -1):
                    for e in range(len(self.fast.parts)):
                        for _ in range(2):
                            self.reset()
                            for s, p in zip(samples, prepared):
                                out = self.run_batch([(k, dict(s, expert=e), p, 0.0) for k in range(b)])
                self.warmup_timing = out[0]["timing"]
                self.reset()
        self.n = 0

    def reset(self, sim=None):
        """forget cached ViT outputs: the single path's (sim None) or one simulator's; run_batch's all with sim None."""
        if sim is None:
            self.fast.vit_cache.clear(); self.fast.caches.clear()
        else:
            self.fast.caches.pop(sim, None)

    def load_lora(self, path, version=None):
        """merge a LoRA file into the last expert set (the section expert, or the only one; with only one also the LLM)
        in place -> seconds taken."""
        import engine
        t = time.perf_counter()
        lora, scale, meta_version = engine.read_lora(path)
        if len(self.fast.parts) > 1 and any(n.startswith(engine.LLM) for n in lora):
            raise ValueError("%s: LLM LoRA with --expert2 (both expert sets share the LLM)" % path)
        root = self.fast.parts[-1] if len(self.fast.parts) > 1 else self.model     # one set: its modules are the model's
        self.lora_names = engine.apply_lora(root, self.base, lora, scale, self.lora_names)
        v = meta_version if version is None else version
        self.lora_version = int(v) if str(v).isdigit() else v
        return time.perf_counter() - t

    def poll_lora(self):
        """a new latest.json (mtime changed) -> load_lora of its version; returns the seconds, or None when unchanged."""
        try:
            mtime = os.stat(self.lora_watch).st_mtime_ns
        except (TypeError, FileNotFoundError):
            return None
        if mtime == self.lora_mtime:
            return None
        self.lora_mtime = mtime
        latest = json.loads(open(self.lora_watch).read())
        if latest["version"] == self.lora_version:
            return None
        return self.load_lora(os.path.join(os.path.dirname(self.lora_watch), latest["path"]), latest["version"])

    def expert_of(self, sample):
        return min(int(sample.get("expert", 0)), len(self.fast.parts) - 1)   # no second set loaded: the base

    def prepare(self, sample):
        from prep_proc import cpu_prepare                    # the same calls the --prep process worker makes
        return self.pin(cpu_prepare(sample, self.model.config, self.fast.image_token, self.fast.vlm.visual.spatial_merge_size))

    def pin(self, prepared):
        (b, keys, lay), timing = prepared
        b["tokenized_data"]["pixel_values"] = b["tokenized_data"]["pixel_values"].pin_memory()   # async copies of new images
        return prepared

    def _dev(self, b):
        dev = lambda x: {k: dev(v) if isinstance(v, dict) else v if k == "pixel_values" else v.to(self.device)
                         if torch.is_tensor(v) else v for k, v in x.items()}
        return dev(b)                                          # pixel_values stay pinned on the CPU (FastEngine copies new ones)

    def _x0(self):
        if self.seed is not None:
            torch.manual_seed(int(self.seed) + self.n)
        self.n += 1
        return torch.randn(1, *self.model.action_space.get_action_space_dims(), device=self.device)   # as FlowMatching._euler

    def _post(self, act, hx, hr):
        """normalised action (1, 64, 2) on the CPU -> accel, kappa, v0 (and xyz) on the CPU-side action space."""
        sp, hxc, hrc = self.sp_cpu, hx.cpu(), hr.cpu()
        accel = act[0, :, 0] * sp.accel_std.float() + sp.accel_mean.float()
        kappa = act[0, :, 1] * sp.curvature_std.float() + sp.curvature_mean.float()
        v0 = sp.estimate_t0_states(hxc[:, 0].float(), hrc[:, 0].float())["v"]
        xyz = sp.action_to_traj(act, hxc[:, 0], hrc[:, 0])[0][0].float().numpy() if self.want_xyz else None   # the node
        return dict(accel=accel.numpy().astype(np.float64), kappa=kappa.numpy().astype(np.float64),           # drives on (a, kappa)
                    v0=float(v0.reshape(-1)[0]), xyz=xyz)

    def run(self, sample, prepared):
        (b, keys, lay), timing = prepared
        t1 = time.perf_counter()
        b = self._dev(b)
        x0 = self._x0()
        hx, hr, e = b["ego_history_xyz"], b["ego_history_rot"], self.expert_of(sample)
        act, parts = self.fast.run(b["tokenized_data"], hx, hr, x0, keys, lay, expert=e)
        t2 = time.perf_counter()
        out = self._post(act.cpu(), hx, hr)                     # one sync; the rest on the CPU
        t3 = time.perf_counter()
        return dict(out, expert=e, timing=dict(timing, model_ms=round(1e3 * (t3 - t1), 1), engine_ms=round(1e3 * (t2 - t1), 1),
                                               post_ms=round(1e3 * (t3 - t2), 1), **parts))

    def run_batch(self, jobs):
        """several simulators' requests at once (model_server --sims): jobs (sim, sample, prepared, sigma) with at most
        one per simulator; sigma = exploration std per Euler step (scalar or (steps,); 0 = deterministic). -> outputs as
        run(), and where some sigma_k > 0 the RL sampler's record (Alpamayo-Cosmos train/rl/policy.py Policy.sample):
        chain x_0..x_K (steps + 1, 64, 2) normalised, sigma (steps,), logp (steps,) = log N(x_{k+1}; x_k + dt v(x_k,
        t_k), sigma_k^2 I) (0 where sigma_k = 0; the x_0 term is policy-independent and left out), logp_sum.
        Requests on different expert sets run as one group each (one expert graph per group)."""
        t1 = time.perf_counter()
        items = []
        for sim, sample, ((b, keys, lay), timing), sigma in jobs:
            b = self._dev(b)
            x0 = self._x0()
            eps = torch.randn(self.num_steps, *x0.shape, device=self.device)
            sig = sigmas(sigma, self.num_steps)
            items.append(dict(sim=sim, td=b["tokenized_data"], hx=b["ego_history_xyz"], hr=b["ego_history_rot"], keys=keys, lay=lay,
                              x0=x0, noise=sig.to(self.device).view(-1, 1, 1, 1) * eps, eps=eps, sigma=sig, timing=timing,
                              expert=self.expert_of(sample)))
        res = [None] * len(items)
        for e in sorted({it["expert"] for it in items}):
            grp = [i for i, it in enumerate(items) if it["expert"] == e]
            act, chain, parts = self.fast.run_batch([items[i] for i in grp], expert=e)
            act, chain = act.cpu(), chain.cpu()
            for r, i in enumerate(grp):
                res[i] = (act[r:r + 1], chain[:, r], dict(parts, group=len(grp)))
        t2 = time.perf_counter()
        outs = []
        for it, (act, chain, parts) in zip(items, res):
            out = dict(self._post(act, it["hx"], it["hr"]), expert=it["expert"], lora_version=self.lora_version)
            sig = it["sigma"]
            if (sig > 0).any():
                lp = rl_logp(it["eps"].cpu(), sig)
                out.update(chain=chain.numpy(), sigma=sig.numpy(), logp=lp.numpy(), logp_sum=float(lp.sum()))
            out["timing"] = dict(it["timing"], engine_ms=round(1e3 * (t2 - t1), 1), **parts)
            outs.append(out)
        t3 = time.perf_counter()
        for out in outs:
            out["timing"].update(model_ms=round(1e3 * (t3 - t1), 1), post_ms=round(1e3 * (t3 - t2), 1))
        return outs


def sigmas(sigma, steps):
    """exploration std per Euler step from a scalar or `steps` values: all > 0 (an RL decision: the learner needs every
    step's log-prob) or all 0 (deterministic)."""
    sig = torch.as_tensor(sigma, dtype=torch.float32).expand(steps).clone()
    if not (bool((sig > 0).all()) or bool((sig == 0).all())):
        raise ValueError("sigma %s: all > 0 or all 0" % sig.tolist())
    return sig


def rl_logp(eps, sigma):
    """per Euler step log N(x_{k+1}; mu_k, sigma_k^2 I) of x_{k+1} = mu_k + sigma_k eps_k over the 64 x 2 action
    (eps (steps, ..., 64, 2), sigma (steps,)); 0 where sigma_k = 0 (a deterministic step)."""
    e = eps.reshape(len(sigma), -1).double()
    lp = -0.5 * e.pow(2).sum(1) - e.shape[1] * (sigma.double().clamp_min(1e-30).log() + 0.5 * math.log(2 * math.pi))
    return torch.where(sigma > 0, lp, torch.zeros_like(lp)).float()
