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
"""
import copy
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
    __init__ warms up on stored requests (model_server --dump files) before the server listens."""

    def __init__(self, ckpt, vlm="Qwen/Qwen3-VL-4B-Instruct", num_steps=10, seed=None, device="cuda:0", warmup=(), pad_to=1792,
                 want_xyz=False):
        import engine
        from fast_engine import FastEngine, fingerprints, layout
        t = time.perf_counter()
        self.model = engine.load(ckpt, vlm=vlm, device=device)
        self.fast, self.fingerprints, self.layout = FastEngine(self.model, num_steps=num_steps, pad_to=pad_to), fingerprints, layout
        self.device, self.num_steps, self.seed, self.n, self.want_xyz = device, num_steps, seed, 0, want_xyz
        self.sp_cpu = copy.deepcopy(self.model.action_space).cpu()          # output conversion off the GPU
        self.load_s = time.perf_counter() - t
        if warmup:
            from assemble import build_sample
            from e2e_driver import wire
            for i, f in enumerate(list(warmup) * 3):           # compile + capture, then two timed passes; the first file
                sample = build_sample(wire.unpack(open(f, "rb").read()), key=f"warmup/{i}")   # of each round has every
                if i % len(warmup) == 0:                              # image new, the next ones the steady state
                    self.fast.vit_cache.clear()                      # (newest camera frames + BEVs)
                out = self.run(sample, self.prepare(sample))
            self.warmup_timing = out["timing"]
        self.n = 0

    def prepare(self, sample):
        b, timing = super().prepare(sample)
        t = time.perf_counter()
        td = b["tokenized_data"]
        keys = self.fingerprints(td["pixel_values"], td["image_grid_thw"])
        lay = self.layout(td["input_ids"], td["image_grid_thw"], self.fast.image_token, self.fast.vlm.visual.spatial_merge_size)
        td["pixel_values"] = td["pixel_values"].to(torch.bfloat16).pin_memory()   # the ViT input dtype; async copies of new images
        return (b, keys, lay), dict(timing, fingerprint_ms=round(1e3 * (time.perf_counter() - t), 1))

    def run(self, sample, prepared):
        m, ((b, keys, lay), timing) = self.model, prepared
        t1 = time.perf_counter()
        dev = lambda x: {k: dev(v) if isinstance(v, dict) else v if k == "pixel_values" else v.to(self.device)
                         if torch.is_tensor(v) else v for k, v in x.items()}
        b = dev(b)                                             # pixel_values stay pinned on the CPU (FastEngine copies new ones)
        if self.seed is not None:
            torch.manual_seed(int(self.seed) + self.n)
        self.n += 1
        hx, hr = b["ego_history_xyz"], b["ego_history_rot"]
        x0 = torch.randn(1, *m.action_space.get_action_space_dims(), device=self.device)   # as FlowMatching._euler
        act, parts = self.fast.run(b["tokenized_data"], hx, hr, x0, keys, lay)
        t2 = time.perf_counter()
        act = act.cpu()                                        # one sync; the rest on the CPU (the CPU-side action space)
        sp, hxc, hrc = self.sp_cpu, hx.cpu(), hr.cpu()
        accel = act[0, :, 0] * sp.accel_std.float() + sp.accel_mean.float()
        kappa = act[0, :, 1] * sp.curvature_std.float() + sp.curvature_mean.float()
        v0 = sp.estimate_t0_states(hxc[:, 0].float(), hrc[:, 0].float())["v"]
        xyz = sp.action_to_traj(act, hxc[:, 0], hrc[:, 0])[0][0].float().numpy() if self.want_xyz else None   # the node
        t3 = time.perf_counter()                                                                             # drives on (a, kappa)
        return dict(accel=accel.numpy().astype(np.float64), kappa=kappa.numpy().astype(np.float64),
                    v0=float(v0.reshape(-1)[0]), xyz=xyz,
                    timing=dict(timing, model_ms=round(1e3 * (t3 - t1), 1), engine_ms=round(1e3 * (t2 - t1), 1),
                                post_ms=round(1e3 * (t3 - t2), 1), **parts))
