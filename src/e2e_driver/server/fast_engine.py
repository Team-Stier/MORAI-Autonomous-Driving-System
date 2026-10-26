"""Static-shape CUDA-graph inference for the engine model (engine.load): the same computation as
TeacherVLA.sample_trajectories (no CoC, prefix prefill, Euler flow matching), laid out like the 5080 latency study
(winpc ~/claude-bench/bench2.py) but on the real modules and the real request:

  ViT     per image, one CUDA graph per (encoder, image size); an image already seen (same pixels: a camera frame that is
          still in the history window, or an unchanged LiDAR BEV) reuses its cached merger and DeepStack outputs;
  LLM     prefill of the prefix right-padded to pad_to tokens (1792; the prompts are ~1.73k; causal: real tokens never
          see the padding), K/V written into static buffers, DeepStack added at the image tokens after the first layers;
          a longer prefix gets its own length (a multiple of 64), compiled on first use;
  expert  the Euler steps in one graph: action tokens attend to the valid prefix (padding masked) and to each other
          (expert_non_causal_attention), positions after the real prefix as in sample_trajectories.
Runs under bf16 autocast like sample_trajectories (LayerNorms in fp32, the Euler state fp32). Inputs differ from HF's
only by the padding; outputs match the eager engine to bf16 rounding (test/fast_parity.py).
"""
import numpy as np
import torch
import torch.nn.functional as F
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


def _graph(fn):
    st = torch.cuda.Stream()
    st.wait_stream(torch.cuda.current_stream())
    with torch.cuda.stream(st):
        for _ in range(2):
            fn()
    torch.cuda.current_stream().wait_stream(st)
    g = torch.cuda.CUDAGraph()
    with torch.cuda.graph(g):
        fn()
    return g


class FastEngine:
    def __init__(self, model, num_steps=10, use_graphs=True, compile=True, pad_to=1792):
        self.m, self.steps, self.use_graphs, self.pad_to = model, num_steps, use_graphs, pad_to
        # Inductor fuses norms, rotary, activation quantizers and residuals (the study: one compiled function, then the
        # graph); expert: one compiled Euler step reused 10 times inside the graph
        self.compile = (lambda f: torch.compile(f, dynamic=False)) if compile else (lambda f: f)
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
        self.vit_cache, self.vit_graphs, self.llm, self.exp = {}, {}, {}, {}
        self.image_token = model.vlm.config.image_token_id
        self.stats = {}

    # ---------------------------------------------------------------- ViT
    def _vit_fn(self, enc, t, h, w):
        grid = torch.tensor([[t, h, w]], device=self.dev)
        S = t * h * w
        pos = enc.fast_pos_embed_interpolate(grid)
        rot = enc.rot_pos_emb(grid).reshape(S, -1)
        emb = torch.cat((rot, rot), -1)
        cos, sin = emb.cos(), emb.sin()
        ds_idx = list(enc.deepstack_visual_indexes)
        px = torch.zeros(S, enc.patch_embed.proj.weight[0].numel(), device=self.dev, dtype=enc.patch_embed.proj.weight.dtype)
        out = {}

        def body():
            x = enc.patch_embed(px) + pos
            deep = []
            for li, blk in enumerate(enc.blocks):
                a = blk.attn
                q, k, v = a.qkv(blk.norm1(x)).reshape(S, 3, a.num_heads, -1).permute(1, 0, 2, 3).unbind(0)
                q, k = mq.apply_rotary_pos_emb_vision(q, k, cos, sin)
                o = F.scaled_dot_product_attention(q.transpose(0, 1)[None], k.transpose(0, 1)[None], v.transpose(0, 1)[None],
                                                   scale=a.scaling)
                x = x + a.proj(o[0].transpose(0, 1).reshape(S, -1))
                x = x + blk.mlp(blk.norm2(x))
                if li in ds_idx:
                    deep.append(enc.deepstack_merger_list[ds_idx.index(li)](x))
            return enc.merger(x), deep
        body = self.compile(body)

        def fn():
            out["e"], out["d"] = body()
        return px, fn, out

    def _vit(self, lidar, key, px_i):
        enc = self.vlm.lidar_visual if lidar else self.vlm.visual
        size = (lidar, *key[:3])
        if size not in self.vit_graphs:
            px, fn, out = self._vit_fn(enc, *key[:3])
            px.copy_(px_i)
            g = _graph(fn) if self.use_graphs else None
            self.vit_graphs[size] = (px, fn, out, g)
        px, fn, out, g = self.vit_graphs[size]
        px.copy_(px_i)
        g.replay() if g is not None else fn()
        return out["e"].clone(), [d.clone() for d in out["d"]]

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
    def _exp_buffers(self, Lp, llm):
        ec, n = self.expert.config, self.n
        b = dict(x0=torch.zeros(1, n, 2, device=self.dev), x=torch.zeros(1, n, 2, device=self.dev),
                 mask=torch.zeros(1, 1, 1, Lp + n, device=self.dev, dtype=torch.float32),
                 cos=torch.zeros(1, n, self.hd, device=self.dev, dtype=torch.bfloat16),
                 sin=torch.zeros(1, n, self.hd, device=self.dev, dtype=torch.bfloat16))
        nq = ec.num_attention_heads
        g = nq // self.kvh
        m = self.m

        def step(x, t, dt):
            h = m.action_in_proj(x, t)
            for i, layer in enumerate(self.expert.layers):
                a = layer.self_attn
                y = layer.input_layernorm(h)
                q = a.q_norm(a.q_proj(y).view(1, n, nq, self.hd)).transpose(1, 2)
                k = a.k_norm(a.k_proj(y).view(1, n, self.kvh, self.hd)).transpose(1, 2)
                v = a.v_proj(y).view(1, n, self.kvh, self.hd).transpose(1, 2)
                q, k = mq.apply_rotary_pos_emb(q, k, b["cos"], b["sin"])
                llm["K"][i][:, :, Lp:].copy_(k); llm["V"][i][:, :, Lp:].copy_(v)
                # attention as two bf16 batched GEMMs and an fp32 softmax over [prefix | action] keys, GQA folded into
                # the query length (query heads j*g..j*g+g-1 read KV head j). SDPA for 64 queries x ~1.8k keys with the
                # padding mask fell back to fp32 SIMT matmuls (enable_gqa) or a 16-CTA mem-efficient kernel (46 ms)
                qf = q.reshape(1, self.kvh, g * n, self.hd)
                att = (torch.matmul(qf, llm["K"][i].transpose(-1, -2)).float() * a.scaling + b["mask"]).softmax(-1)
                o = torch.matmul(att.to(torch.bfloat16), llm["V"][i]).reshape(1, nq, n, self.hd)
                h = h + a.o_proj(o.transpose(1, 2).reshape(1, n, nq * self.hd))
                h = h + layer.mlp(layer.post_attention_layernorm(h))
            vel = m.action_out_proj(self.expert.norm(h)).view(1, n, 2)
            return x + dt * vel
        step = self.compile(step)

        def fn():
            x = b["x0"]
            for s in range(self.steps):
                x = step(x, self.t_start[s], self.dt[s])
            b["x"].copy_(x)
        assert self.non_causal, "a causal expert needs a causal block in the mask"
        return b, fn

    # ---------------------------------------------------------------- one request
    @torch.inference_mode()
    @torch.autocast("cuda", dtype=torch.bfloat16)          # as sample_trajectories (graphs are captured under it too)
    def run(self, tokenized_data, hx, hr, x0, keys):
        """tokenized_data on the GPU (as sample_trajectories takes it), keys = fingerprints(CPU pixel_values, grid);
        x0 (1, 64, 2) fp32 initial noise. Returns the normalised action (1, 64, 2) fp32 and the per-part times (ms)."""
        m, ev = self.m, [torch.cuda.Event(enable_timing=True) for _ in range(5)]
        ev[0].record()
        ids = m.fuse_traj_tokens(tokenized_data["input_ids"], dict(ego_history_xyz=hx, ego_history_rot=hr))
        assert int(ids[0, -1]) == m.config.traj_token_ids["future_start"]
        grid, lidar = tokenized_data["image_grid_thw"], tokenized_data["image_is_lidar"].bool().tolist()
        px = tokenized_data["pixel_values"]
        n_patch = (grid[:, 0] * grid[:, 1] * grid[:, 2]).tolist()
        embeds, deeps, new, seen = [], [], 0, {}
        for i, (key, px_i) in enumerate(zip(keys, px.split(n_patch))):
            ck = (lidar[i], key)
            if ck not in self.vit_cache:
                self.vit_cache[ck] = self._vit(lidar[i], key, px_i); new += 1
            seen[ck] = self.vit_cache[ck]
            embeds.append(seen[ck][0]); deeps.append(seen[ck][1])
        self.vit_cache = seen                                                # keep what the current window uses
        ev[1].record()
        L = ids.shape[1]
        Lp = max(self.pad_to, -(-L // PAD) * PAD)          # one fixed length: a new length means minutes of compiling
        vis = torch.nonzero(ids[0] == self.image_token).flatten()
        if Lp not in self.llm or self.llm[Lp][0]["vis"].numel() != vis.numel():
            self.llm[Lp] = self._llm_buffers(Lp, vis.numel()) + (None,)
            self.exp[Lp] = self._exp_buffers(Lp, self.llm[Lp][0]) + (None,)
        lb, lfn, lg = self.llm[Lp]
        eb, efn, eg = self.exp[Lp]
        E = self.lm.embed_tokens(ids)
        E[0, vis] = torch.cat(embeds).to(E.dtype)
        pos, deltas = self.vlm.get_rope_index(ids, grid, None, tokenized_data["attention_mask"])
        cos, sin = self.lm.rotary_emb(E, pos)
        lb["E"].zero_(); lb["E"][:, :L].copy_(E)
        lb["cos"][:, :L].copy_(cos); lb["sin"][:, :L].copy_(sin)
        lb["vis"].copy_(vis)
        for j, d in enumerate(lb["deep"]):
            d.copy_(torch.cat([x[j] for x in deeps]))
        n = self.n
        pa = (torch.arange(n, device=self.dev)[None, None] + deltas.view(1, 1, 1) + L).expand(3, 1, n)
        ca, sa = self.expert.rotary_emb(eb["cos"], pa)
        eb["cos"].copy_(ca); eb["sin"].copy_(sa)
        eb["mask"].zero_(); eb["mask"][..., L:Lp] = float("-inf")                    # padded prefix rows
        eb["x0"].copy_(x0)
        if self.use_graphs and lg is None:
            lg = _graph(lfn); eg = _graph(efn)
            self.llm[Lp], self.exp[Lp] = (lb, lfn, lg), (eb, efn, eg)
        ev[2].record()
        lg.replay() if lg is not None else lfn()
        ev[3].record()
        eg.replay() if eg is not None else efn()
        ev[4].record()
        torch.cuda.synchronize()
        t = [ev[i].elapsed_time(ev[i + 1]) for i in range(4)]
        self.stats = dict(vit_ms=round(t[0], 1), assemble_ms=round(t[1], 1), llm_ms=round(t[2], 1), expert_ms=round(t[3], 1),
                          llm_expert_ms=round(t[2] + t[3], 1), new_images=new, prefix_tokens=L, padded=Lp)
        return eb["x"].clone(), self.stats
