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
