"""Static-shape CUDA-graph inference for the engine model (engine.load): the same computation as
TeacherVLA.sample_trajectories (no CoC, prefix prefill, Euler flow matching), laid out like the 5080 latency study
(winpc ~/claude-bench/bench2.py) but on the real modules and the real request:

  ViT     the new images only (normally the newest frame of each camera + the 2 LiDAR BEVs; MODEL_PLAN 5.2), batched per
          encoder (camera FP8, LiDAR BF16) into one fixed slot pattern, one slot per camera (its newest frame's size) or
          per BEV, so one CUDA graph per encoder: fewer new images leave slots unused, more (a jittery camera stream, a
          teleport, the first request) take more passes, never a recompile; an image already seen (same pixels: a camera
          frame still in the history window) reuses its cached merger and DeepStack outputs;
  LLM     prefill of the prefix right-padded to pad_to tokens (1792; the prompts are ~1.73k; causal: real tokens never
          see the padding), K/V written into static buffers, DeepStack added at the image tokens after the first layers;
          a longer prefix gets its own length (a multiple of 64), compiled on first use;
  expert  the Euler steps in one graph: action tokens attend to the valid prefix (padding masked) and to each other
          (expert_non_causal_attention), positions after the real prefix as in sample_trajectories.
Runs under bf16 autocast like sample_trajectories (LayerNorms in fp32, the Euler state fp32). Inputs differ from HF's
only by the padding; outputs match the eager engine to bf16 rounding (test/fast_parity.py).
"""
import time

import numpy as np
import torch
import torch.nn.functional as F
from morai_vla.data import N_FRAMES
from transformers.models.qwen3_vl import modeling_qwen3_vl as mq

PAD = 64


def fingerprints(pixel_values, grid):
    """per image: (rows, wrapped sum of the float bits, sum of every 7th word); identical images give identical keys."""
    px = pixel_values.detach().cpu().contiguous()
    words = px.view(torch.int32).numpy().reshape(-1)
    per = px.shape[1]
    keys, at = [], 0
    for t, h, w in grid.tolist():
        n = t * h * w * per
        seg = words[at:at + n].astype(np.int64)
        keys.append((t, h, w, int(seg.sum()), int(seg[::7].sum())))
        at += n
    return keys


def layout(input_ids, grid, image_token, merge):
    """M-RoPE positions of a one-sample prefix with images only, as Qwen3VLModel.get_rope_index computes them (text
    positions continue after each image at its start + max(grid h, w) / merge), but vectorised on the CPU so it runs in
    the prepare thread: -> positions (3, 1, L), rope delta (1, 1), image token indices."""
    ids = input_ids.reshape(-1).cpu().numpy()
    L = len(ids)
    vis = np.flatnonzero(ids == image_token)
    starts = vis[np.r_[True, np.diff(vis) > 1]] if len(vis) else vis
    pos, st, prev = np.zeros((3, L), np.int64), 0, 0
    for a, (t, h, w) in zip(starts.tolist(), grid.tolist()):
        gh, gw = h // merge, w // merge
        n = t * gh * gw
        pos[:, prev:a] = np.arange(st, st + a - prev); st += a - prev
        pos[0, a:a + n] = st + np.repeat(np.arange(t), gh * gw)
        pos[1, a:a + n] = st + np.tile(np.repeat(np.arange(gh), gw), t)
        pos[2, a:a + n] = st + np.tile(np.arange(gw), t * gh)
        st, prev = st + max(t, gh, gw), a + n
    pos[:, prev:] = np.arange(st, st + L - prev)
    return torch.from_numpy(pos)[:, None], torch.tensor([[int(pos.max()) + 1 - L]]), torch.from_numpy(vis)


def _graph(fn):
    st = torch.cuda.Stream()
    st.wait_stream(torch.cuda.current_stream())
    with torch.cuda.stream(st):
        for _ in range(2):
            fn()
    torch.cuda.current_stream().wait_stream(st)
    g = torch.cuda.CUDAGraph()
    # captured on its own stream: cuBLAS/cuBLASLt workspaces are per (handle, capture stream), and torch.cuda.graph's
    # shared default capture stream gave the camera and LiDAR ViT graphs one workspace; replayed at once on two streams
    # they raced on it and hung the GPU (a few in 10 multi-pass steps; 0 in 400 after this)
    with torch.cuda.graph(g, stream=st):
        fn()
    return g


def _euler_step(self, m, b, llm, B, n, nq, g, Lp):
    """one Euler step of the expert over B rows (FastEngine._exp_buffers); self = the FastEngine (or a stand-in with
    expert, hd, kvh) and m = the model (or one with action_in_proj, action_out_proj): the pre-RL closure names."""
    def step(x, t, dt):
        h = m.action_in_proj(x, t)
        for i, layer in enumerate(self.expert.layers):
            a = layer.self_attn
            y = layer.input_layernorm(h)
            q = a.q_norm(a.q_proj(y).view(B, n, nq, self.hd)).transpose(1, 2)
            k = a.k_norm(a.k_proj(y).view(B, n, self.kvh, self.hd)).transpose(1, 2)
            v = a.v_proj(y).view(B, n, self.kvh, self.hd).transpose(1, 2)
            q, k = mq.apply_rotary_pos_emb(q, k, b["cos"], b["sin"])
            llm["K"][i][:, :, Lp:].copy_(k); llm["V"][i][:, :, Lp:].copy_(v)
            # attention as two bf16 batched GEMMs and an fp32 softmax over [prefix | action] keys, GQA folded into
            # the query length (query heads j*g..j*g+g-1 read KV head j). Measured alternatives for 64 queries x
            # ~1.8k keys with the padding: masked SDPA fell back to fp32 SIMT (enable_gqa) or a 16-CTA mem-efficient
            # kernel (46 ms), cuDNN 2x slower, varlen flash (exact lengths, no mask) 64 vs 44 ms (16 CTAs, no split-KV),
            # FlexAttention (every key block partial for a variable L) 330 us a call vs ~35 us
            qf = q.reshape(B, self.kvh, g * n, self.hd)
            att = (torch.matmul(qf, llm["K"][i].transpose(-1, -2)).float() * a.scaling + b["mask"]).softmax(-1)
            o = torch.matmul(att.to(torch.bfloat16), llm["V"][i]).reshape(B, nq, n, self.hd)
            h = h + a.o_proj(o.transpose(1, 2).reshape(B, n, nq * self.hd))
            h = h + layer.mlp(layer.post_attention_layernorm(h))
        vel = m.action_out_proj(self.expert.norm(h)).view(B, n, 2)
        return x + dt * vel
    return step


class FastEngine:
    def __init__(self, model, num_steps=10, use_graphs=True, compile=True, pad_to=1792, batch=1):
        self.m, self.steps, self.use_graphs, self.pad_to, self.batch = model, num_steps, use_graphs, pad_to, batch
        # Inductor fuses norms, rotary, activation quantizers and residuals (the study: one compiled function, then the
        # graph); expert: one compiled Euler step reused 10 times inside the graph
        self.compile = (lambda f, **k: torch.compile(f, dynamic=False, **k)) if compile else (lambda f, **k: f)
        vlm = model.vlm.model
        self.vlm, self.lm, self.expert = vlm, vlm.language_model, model.expert
        tc, ec = self.lm.config, self.expert.config
        self.dev = next(self.lm.parameters()).device
        self.n = model.action_space.get_action_space_dims()[0]
        self.kvh, self.hd = tc.num_key_value_heads, tc.head_dim
        assert (ec.num_key_value_heads, ec.head_dim, ec.num_hidden_layers) == (self.kvh, self.hd, tc.num_hidden_layers)
        self.non_causal = bool(model.config.expert_non_causal_attention)
        ts = torch.linspace(0.0, 1.0, num_steps + 1, device=self.dev)            # as FlowMatching._euler
        self.t_start = [ts[i].view(1, 1, 1).clone() for i in range(num_steps)]
        self.dt = [(ts[i + 1] - ts[i]).view(1, 1, 1).clone() for i in range(num_steps)]
        self.vit_cache, self.vit_graphs, self.llm, self.exp, self.rope = {}, {}, {}, {}, {}
        self.caches, self.bkv, self.bexp = {}, {}, {}           # run_batch: ViT cache per simulator, batch K/V, graphs
        # expert sets: [0] the model's own; EngineBackend adds a LoRA'd copy (expert2; RL_PLAN 5 A, picked per step)
        self.parts = [torch.nn.ModuleDict(dict(expert=model.expert, action_in_proj=model.action_in_proj, action_out_proj=model.action_out_proj))]
        self.image_token = model.vlm.config.image_token_id
        self.side = torch.cuda.Stream(self.dev)
        self.stats = {}

    # ---------------------------------------------------------------- ViT
    def _vit_fn(self, enc, sizes):
        """one pass over several images of one encoder (bench2's vit_forward): patches concatenated, attention per image."""
        S = [t * h * w for t, h, w in sizes]
        grid = torch.tensor(sizes, device=self.dev)
        pos = enc.fast_pos_embed_interpolate(grid)
        rot = enc.rot_pos_emb(grid).reshape(sum(S), -1)
        emb = torch.cat((rot, rot), -1)
        cos, sin = emb.cos(), emb.sin()
        spans = np.cumsum([0] + S).tolist()
        ds_idx = list(enc.deepstack_visual_indexes)
        px = torch.zeros(sum(S), enc.patch_embed.proj.weight[0].numel(), device=self.dev, dtype=enc.patch_embed.proj.weight.dtype)
        out = {}

        def body():
            x = enc.patch_embed(px) + pos
            deep = []
            for li, blk in enumerate(enc.blocks):
                a = blk.attn
                q, k, v = a.qkv(blk.norm1(x)).reshape(sum(S), 3, a.num_heads, -1).permute(1, 0, 2, 3).unbind(0)
                q, k = mq.apply_rotary_pos_emb_vision(q, k, cos, sin)
                q, k, v = q.transpose(0, 1)[None], k.transpose(0, 1)[None], v.transpose(0, 1)[None]
                o = torch.cat([F.scaled_dot_product_attention(q[:, :, a0:a1], k[:, :, a0:a1], v[:, :, a0:a1], scale=a.scaling)
                               for a0, a1 in zip(spans, spans[1:])], 2)
                x = x + a.proj(o[0].transpose(0, 1).reshape(sum(S), -1))
                x = x + blk.mlp(blk.norm2(x))
                if li in ds_idx:
                    deep.append(enc.deepstack_merger_list[ds_idx.index(li)](x))
            return enc.merger(x), deep
        body = self.compile(body)

        def fn():
            out["e"], out["d"] = body()
        return px, fn, out, [n // enc.spatial_merge_size ** 2 for n in S], list(zip(spans, spans[1:]))

    def _vit(self, lidar, sizes, pxs):
        """ViT outputs (merger, DeepStack list) of one pass over the slot pattern `sizes` of one encoder; pxs per slot,
        None for an unused slot (its stale pixels run along, attention is per image, the output is dropped)."""
        enc = self.vlm.lidar_visual if lidar else self.vlm.visual
        if (lidar, sizes) not in self.vit_graphs:
            px, fn, out, ntok, spans = self._vit_fn(enc, sizes)
            g = _graph(fn) if self.use_graphs else None
            self.vit_graphs[(lidar, sizes)] = (px, fn, out, ntok, spans, g)
        px, fn, out, ntok, spans, g = self.vit_graphs[(lidar, sizes)]
        for (a0, a1), p in zip(spans, pxs):
            if p is not None:
                px[a0:a1].copy_(p)
        g.replay() if g is not None else fn()
        es, ds = out["e"].split(ntok), [d.split(ntok) for d in out["d"]]
        return [(es[i].clone(), [d[i].clone() for d in ds]) if p is not None else None for i, p in enumerate(pxs)]

    # ---------------------------------------------------------------- LLM prefill
    def _llm_buffers(self, Lp, n_vis):
        tc = self.lm.config
        b = dict(E=torch.zeros(1, Lp, tc.hidden_size, device=self.dev, dtype=torch.bfloat16),
                 cos=torch.zeros(1, Lp, self.hd, device=self.dev, dtype=torch.bfloat16),
                 sin=torch.zeros(1, Lp, self.hd, device=self.dev, dtype=torch.bfloat16),
                 vis=torch.zeros(n_vis, device=self.dev, dtype=torch.long),
                 deep=[torch.zeros(n_vis, tc.hidden_size, device=self.dev, dtype=torch.bfloat16) for _ in self.vlm.visual.deepstack_visual_indexes],
                 K=[torch.zeros(1, self.kvh, Lp + self.n, self.hd, device=self.dev, dtype=torch.bfloat16) for _ in self.lm.layers],
                 V=[torch.zeros(1, self.kvh, Lp + self.n, self.hd, device=self.dev, dtype=torch.bfloat16) for _ in self.lm.layers])
        nq = tc.num_attention_heads

        def fn():
            h = b["E"].clone()
            for i, layer in enumerate(self.lm.layers):
                a = layer.self_attn
                x = layer.input_layernorm(h)
                q = a.q_norm(a.q_proj(x).view(1, Lp, nq, self.hd)).transpose(1, 2)
                k = a.k_norm(a.k_proj(x).view(1, Lp, self.kvh, self.hd)).transpose(1, 2)
                v = a.v_proj(x).view(1, Lp, self.kvh, self.hd).transpose(1, 2)
                q, k = mq.apply_rotary_pos_emb(q, k, b["cos"], b["sin"])
                b["K"][i][:, :, :Lp].copy_(k); b["V"][i][:, :, :Lp].copy_(v)
                o = F.scaled_dot_product_attention(q, k, v, is_causal=True, enable_gqa=True, scale=a.scaling)
                h = h + a.o_proj(o.transpose(1, 2).reshape(1, Lp, nq * self.hd))
                h = h + layer.mlp(layer.post_attention_layernorm(h))
                if i < len(b["deep"]):
                    h[0].index_add_(0, b["vis"], b["deep"][i])
        return b, self.compile(fn)

    # ---------------------------------------------------------------- expert Euler steps
    def _exp_buffers(self, Lp, llm, B=1, parts=None, rl=False):
        """the Euler steps of one expert set (parts, default the model's own) over B rows reading llm's K/V (B rows).
        rl: after each step add noise[s] (sigma_s * eps_s, zeros = deterministic) and keep the chain x_0..x_K (the RL
        sampler, RL_PLAN 3.1; spec: Alpamayo-Cosmos train/rl/policy.py Policy.sample)."""
        ec, n = self.expert.config, self.n
        b = dict(x0=torch.zeros(B, n, 2, device=self.dev), x=torch.zeros(B, n, 2, device=self.dev),
                 mask=torch.zeros(B, 1, 1, Lp + n, device=self.dev, dtype=torch.float32),
                 cos=torch.zeros(B, n, self.hd, device=self.dev, dtype=torch.bfloat16),
                 sin=torch.zeros(B, n, self.hd, device=self.dev, dtype=torch.bfloat16))
        if rl:
            b.update(noise=torch.zeros(self.steps, B, n, 2, device=self.dev), chain=torch.zeros(self.steps + 1, B, n, 2, device=self.dev))
        nq = ec.num_attention_heads
        g = nq // self.kvh
        P = self.parts[0] if parts is None else parts
        # the model's own set traces through the closure names of the pre-RL step (self.expert, m.action_*): the same FX
        # graph, so the deployment path keeps its cached compiled kernels (renamed closures compiled and autotuned anew
        # and moved actions by up to 2e-4, 10/27); another set goes through stand-ins with the same attributes
        if P is self.parts[0]:
            sf, m = self, self.m
        else:                                     # the ModuleDict has expert / action_in_proj / action_out_proj
            P.hd, P.kvh = self.hd, self.kvh
            sf = m = P
        t_start = self.t_start if B == 1 else [t.expand(B, 1, 1) for t in self.t_start]   # action_in_proj cats per row
        step = _euler_step(sf, m, b, llm, B, n, nq, g, Lp)
        # max-autotune times Triton templates against cuBLAS for the bf16 attention GEMMs (cuBLAS picks 32x32 WMMA kernels
        # for 64 queries x ~1.8k keys): expert 44.2 -> 39.8 ms (the 10/24 benchmark: 39.5); minutes once, then cached
        step = self.compile(step, mode="max-autotune-no-cudagraphs")

        def fn():
            x = b["x0"]
            if rl:
                b["chain"][0].copy_(x)
            for s in range(self.steps):
                x = step(x, t_start[s], self.dt[s])
                if rl:                                      # x_{k+1} = x_k + dt v(x_k, t_k) + sigma_k eps_k
                    x = x + b["noise"][s]
                    b["chain"][s + 1].copy_(x)
            b["x"].copy_(x)
        assert self.non_causal, "a causal expert needs a causal block in the mask"
        return b, fn

    def _exp_row(self, eb, r, L, Lp, deltas, x0):
        """row r of an expert graph's inputs: action-token rotary after the real prefix (L tokens), the padded prefix
        rows L..Lp masked, the initial noise."""
        n = self.n
        pa = (torch.arange(n, device=self.dev)[None, None] + deltas.view(1, 1, 1) + L).expand(3, 1, n)
        ca, sa = self.expert.rotary_emb(eb["cos"][r:r + 1], pa)
        eb["cos"][r:r + 1].copy_(ca); eb["sin"][r:r + 1].copy_(sa)
        eb["mask"][r].zero_(); eb["mask"][r, ..., L:Lp] = float("-inf")                 # padded prefix rows
        eb["x0"][r:r + 1].copy_(x0)

    # ---------------------------------------------------------------- one request
    def _vit_embeds(self, tokenized_data, hx, hr, keys, lay, cache):
        """ViT of the images not in `cache` (one simulator's ViT cache, pruned in place to the current window) -> fused
        prefix ids, per-image merger outputs and DeepStack lists, the layout, the number of new images."""
        m = self.m
        if lay is None:
            lay = layout(tokenized_data["input_ids"], tokenized_data["image_grid_thw"], self.image_token, self.vlm.visual.spatial_merge_size)
        ids = m.fuse_traj_tokens(tokenized_data["input_ids"].to(self.dev), dict(ego_history_xyz=hx, ego_history_rot=hr))
        grid, lidar = tokenized_data["image_grid_thw"], tokenized_data["image_is_lidar"].bool().tolist()
        px = tokenized_data["pixel_values"]                       # may stay on the CPU: only new images are copied over
        n_patch = (grid[:, 0] * grid[:, 1] * grid[:, 2]).tolist()
        pxs = px if isinstance(px, list) else px.split(n_patch)          # a list: per image (assemble records)
        ck = [(lidar[i], k) for i, k in enumerate(keys)]
        new, main = 0, torch.cuda.current_stream()
        self.side.wait_stream(main)
        for enc_lidar in (False, True):           # new images: slot-pattern passes per encoder; LiDAR on a side stream,
            enc_idx = [i for i, c in enumerate(ck) if c[0] == enc_lidar]                    # beside the camera ViT
            # cameras come as N_FRAMES per camera, oldest first (assemble.py): a slot per camera's newest frame
            pattern = tuple(tuple(keys[i][:3]) for i in (enc_idx if enc_lidar else enc_idx[N_FRAMES - 1::N_FRAMES]))
            todo = [i for i in enc_idx if ck[i] not in cache]
            new += len(todo)
            with torch.cuda.stream(self.side if enc_lidar else main):
                while todo:
                    slots = [None] * len(pattern)
                    for i in list(todo):
                        s = next((j for j, z in enumerate(pattern) if slots[j] is None and z == tuple(keys[i][:3])), None)
                        if s is not None:
                            slots[s] = i; todo.remove(i)
                    assert any(j is not None for j in slots), f"image size outside the slot pattern {pattern}"
                    res = self._vit(enc_lidar, pattern, [None if i is None else pxs[i].to(self.dev, torch.bfloat16, non_blocking=True) for i in slots])
                    for i, r in zip(slots, res):
                        if i is not None:
                            cache[ck[i]] = r
        main.wait_stream(self.side)
        keep = {c: cache[c] for c in ck}                                     # keep what the current window uses
        cache.clear(); cache.update(keep)
        return ids, [cache[c][0] for c in ck], [cache[c][1] for c in ck], lay, new

    def _llm_inputs(self, ids, embeds, deeps, lay):
        """the LLM prefill's static inputs for one prefix; its graph is captured on first use -> (buffers, fn, graph, L,
        Lp, rope delta)."""
        L = ids.shape[1]
        Lp = max(self.pad_to, -(-L // PAD) * PAD)          # one fixed length: a new length means minutes of compiling
        pos, deltas, vis = (x.to(self.dev, non_blocking=True) for x in lay)
        if Lp not in self.llm or self.llm[Lp][0]["vis"].numel() != vis.numel():
            self.llm[Lp] = self._llm_buffers(Lp, vis.numel()) + (None,)
            self.exp = {k: v for k, v in self.exp.items() if k[0] != Lp}             # they read the old K/V buffers
        lb, lfn, lg = self.llm[Lp]
        E = self.lm.embed_tokens(ids)
        E[0, vis] = torch.cat(embeds).to(E.dtype)
        cos, sin = self.lm.rotary_emb(E, pos)
        lb["E"].zero_(); lb["E"][:, :L].copy_(E)
        lb["cos"][:, :L].copy_(cos); lb["sin"][:, :L].copy_(sin)
        lb["vis"].copy_(vis)
        for j, d in enumerate(lb["deep"]):
            d.copy_(torch.cat([x[j] for x in deeps]))
        if self.use_graphs and lg is None:
            lg = _graph(lfn)
            self.llm[Lp] = (lb, lfn, lg)
        return lb, lfn, lg, L, Lp, deltas

    @torch.inference_mode()
    @torch.autocast("cuda", dtype=torch.bfloat16)          # as sample_trajectories (graphs are captured under it too)
    def run(self, tokenized_data, hx, hr, x0, keys, lay=None, expert=0):
        """tokenized_data as sample_trajectories takes it (pixel_values may stay on the CPU), keys = fingerprints(CPU
        pixel_values, grid), lay = layout(...) from the prepare thread (else computed here); x0 (1, 64, 2) fp32 initial
        noise; expert = index into self.parts. Returns the normalised action (1, 64, 2) fp32 and the per-part times (ms)."""
        ev = [torch.cuda.Event(enable_timing=True) for _ in range(5)]
        ev[0].record()
        ids, embeds, deeps, lay, new = self._vit_embeds(tokenized_data, hx, hr, keys, lay, self.vit_cache)
        ev[1].record(); c1 = time.perf_counter()
        lb, lfn, lg, L, Lp, deltas = self._llm_inputs(ids, embeds, deeps, lay)
        if (Lp, expert) not in self.exp:
            self.exp[(Lp, expert)] = self._exp_buffers(Lp, lb, parts=self.parts[expert]) + (None,)
        eb, efn, eg = self.exp[(Lp, expert)]
        self._exp_row(eb, 0, L, Lp, deltas, x0)
        if self.use_graphs and eg is None:
            eg = _graph(efn)
            self.exp[(Lp, expert)] = (eb, efn, eg)
        ev[2].record(); c2 = time.perf_counter()
        lg.replay() if lg is not None else lfn()
        ev[3].record()
        eg.replay() if eg is not None else efn()
        ev[4].record()
        torch.cuda.synchronize()
        t = [ev[i].elapsed_time(ev[i + 1]) for i in range(4)]
        self.stats = dict(cpu_assemble_ms=round(1e3 * (c2 - c1), 1), vit_ms=round(t[0], 1), gpu_assemble_ms=round(t[1], 1), llm_ms=round(t[2], 1), expert_ms=round(t[3], 1),
                          llm_expert_ms=round(t[2] + t[3], 1), new_images=new, prefix_tokens=L, padded=Lp)
        return eb["x"].clone(), self.stats

    # ---------------------------------------------------------------- several simulators (RL actor)
    @torch.inference_mode()
    @torch.autocast("cuda", dtype=torch.bfloat16)
    def run_batch(self, items, expert=0):
        """one request per simulator, all on expert set `expert` (model_server --sims). items: dicts sim (its ViT cache
        key), td, hx, hr, keys, lay as run() takes them, x0 (1, 64, 2), noise (steps, 1, 64, 2) = sigma_k eps_k added
        after each Euler step (zeros: deterministic). Each request's ViT and LLM prefill run on the single path's graphs
        with its simulator's cache; its K/V go to its row of the batch K/V; then one expert graph runs the Euler steps of
        all rows (M = 64 x rows in the expert GEMMs). -> actions (b, 64, 2), chain x_0..x_K (steps + 1, b, 64, 2) fp32,
        per-part times (ms)."""
        b = len(items)
        assert 0 < b <= self.batch, f"{b} requests for a batch of {self.batch}"
        ev = [torch.cuda.Event(enable_timing=True) for _ in range(3)]
        ev[0].record()
        Lp = max(max(self.pad_to, -(-it["td"]["input_ids"].shape[1] // PAD) * PAD) for it in items)
        if Lp not in self.bkv:
            kv = lambda: [torch.zeros(self.batch, self.kvh, Lp + self.n, self.hd, device=self.dev, dtype=torch.bfloat16) for _ in self.lm.layers]
            self.bkv[Lp] = dict(K=kv(), V=kv())
        kv = self.bkv[Lp]
        if (Lp, b, expert) not in self.bexp:          # rows 0..b-1 of the batch K/V (every group runs on them in turn)
            rows = {k: [t[:b] for t in v] for k, v in kv.items()}
            self.bexp[(Lp, b, expert)] = self._exp_buffers(Lp, rows, B=b, parts=self.parts[expert], rl=True) + (None,)
        eb, efn, eg = self.bexp[(Lp, b, expert)]
        new = []
        for r, it in enumerate(items):
            ids, embeds, deeps, lay, k = self._vit_embeds(it["td"], it["hx"], it["hr"], it["keys"], it.get("lay"), self.caches.setdefault(it["sim"], {}))
            new.append(k)
            lb, lfn, lg, L, Li, deltas = self._llm_inputs(ids, embeds, deeps, lay)
            lg.replay() if lg is not None else lfn()
            for K, Kb in ((lb["K"], kv["K"]), (lb["V"], kv["V"])):
                torch._foreach_copy_([t[r, :, :Li] for t in Kb], [t[0, :, :Li] for t in K])
            self._exp_row(eb, r, L, Lp, deltas, it["x0"])
            eb["noise"][:, r:r + 1].copy_(it["noise"])
        ev[1].record()
        if self.use_graphs and eg is None:
            eg = _graph(efn)
            self.bexp[(Lp, b, expert)] = (eb, efn, eg)
        eg.replay() if eg is not None else efn()
        ev[2].record()
        torch.cuda.synchronize()
        stats = dict(prefill_ms=round(ev[0].elapsed_time(ev[1]), 1), expert_ms=round(ev[1].elapsed_time(ev[2]), 1),
                     batch=b, new_images=new, padded=Lp)
        return eb["x"].clone(), eb["chain"].clone(), stats
