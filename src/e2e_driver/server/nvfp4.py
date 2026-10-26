"""NVFP4 and FP8 linears for the RTX 5080 (sm_120) engine: real low-precision GEMMs that reproduce the student's QAD
fake quantization (NVIDIA ModelOpt 0.47, morai_vla/qad.py) with the scales it trained with.

From the 5080 NVFP4 study (winpc ~/claude-bench/nvfp4.py, NVFP4_REPORT.md, 10/24): a Triton activation quantizer
writing e4m3 block scales straight into cuBLAS's 128x4 swizzled layout, and torch._scaled_mm on float4_e2m1fn_x2
(cuBLASLt block-scaled FP4; works in torch 2.8 cu128 and 2.14 cu130, 10/26). Changed for the engine:
  - global scales from the QAD quantizers (quant_manifest.json amax): weight amax / (6 * 448), input amax / (6 * 448),
    instead of the weight's own amax and a placeholder 1.0;
  - a block whose decoded scale is below 1e-5 quantizes to zero, as ModelOpt's fp4_fake_quant_block does (it divides
    by 1.0 there, which rounds such tiny values to 0).
ModelOpt rounds |x| / scale to E2M1 with the same boundaries (<= 0.25, < 0.75, <= 1.25, < 1.75, <= 2.5, < 3.5, <= 5);
its division is Triton's approximate one, so a rare exact tie can round the other way (the study: 0.16 % vs torchao).
FP8 (per tensor, e4m3): scales weight amax / 448 and input amax / 448, the ModelOpt FP8 quantizer's.
"""
import torch
import torch.nn.functional as F
import triton
import triton.language as tl

F4, E4 = torch.float4_e2m1fn_x2, torch.float8_e4m3fn
TINY = 1e-5                                         # ModelOpt: decoded block scales below this count as zero blocks
rup = lambda x, m: (x + m - 1) // m * m


@triton.jit
def _quant_nvfp4(x_ptr, q_ptr, s_ptr, g_ptr, M, K, NCB):
    # one program = one 128-row x 64-col tile = one 128x4 swizzled scale block
    pm, pk = tl.program_id(0), tl.program_id(1)
    r = pm * 128 + tl.arange(0, 128)
    c = pk * 64 + tl.arange(0, 64)
    m = (r[:, None] < M) & (c[None, :] < K)
    x = tl.load(x_ptr + r[:, None] * K + c[None, :], mask=m, other=0.0).to(tl.float32)
    xb = tl.reshape(x, (128, 4, 16))
    g = tl.load(g_ptr)  # global decode scale
    # div_rn: Triton's default fp32 division is approximate and flips rounding ties vs torch
    s = tl.minimum(tl.div_rn(tl.max(tl.abs(xb), 2), 6.0 * g), 448.0).to(tl.float8e4nv)
    d = s.to(tl.float32) * g
    a = tl.where(d[:, :, None] < 1e-5, 0.0, tl.div_rn(tl.abs(xb), tl.where(d < 1e-5, 1.0, d)[:, :, None]))
    # round-to-nearest onto {0, .5, 1, 1.5, 2, 3, 4, 6} with ModelOpt's boundaries, saturating
    code = ((a > 0.25).to(tl.uint8) + (a >= 0.75).to(tl.uint8) + (a > 1.25).to(tl.uint8) + (a >= 1.75).to(tl.uint8)
            + (a > 2.5).to(tl.uint8) + (a >= 3.5).to(tl.uint8) + (a > 5.0).to(tl.uint8))
    code = code | tl.where(xb < 0, 8, 0).to(tl.uint8)
    lo, hi = tl.split(tl.reshape(code, (128, 32, 2)))
    cq = pk * 32 + tl.arange(0, 32)
    tl.store(q_ptr + r[:, None] * (K // 2) + cq[None, :], lo | (hi << 4), mask=(r[:, None] < M) & (cq[None, :] < K // 2))
    # cuBLAS block-scale layout: tile (pm, pk) holds 512 bytes, row r -> (r % 32) * 16 + (r // 32) * 4 + col
    rr = tl.arange(0, 128)
    off = (pm * NCB + pk) * 512 + (rr % 32)[:, None] * 16 + (rr // 32)[:, None] * 4 + tl.arange(0, 4)[None, :]
    tl.store(s_ptr + off, s.to(tl.uint8, bitcast=True))


def quant_nvfp4(x, g):
    """x (M, K) bf16 contiguous, g fp32 scalar tensor -> packed fp4 (M, K/2), swizzled e4m3 scales (flat)."""
    M, K = x.shape
    assert K % 64 == 0, K
    q = torch.empty(M, K // 2, device=x.device, dtype=torch.uint8)
    s = torch.empty(rup(M, 128) * (K // 16), device=x.device, dtype=torch.uint8)
    _quant_nvfp4[(rup(M, 128) // 128, K // 64)](x, q, s, g, M, K, K // 64, num_warps=4)
    return q.view(F4), s.view(E4)


def quant_nvfp4_torch(x, g):
    """Same result as quant_nvfp4 in plain torch ops, so torch.compile can fuse it into the op that produces x."""
    M, K = x.shape
    xb = x.float().view(M, K // 16, 16)
    s = (xb.abs().amax(-1, keepdim=True) / (6 * g)).clamp(max=448).to(E4)
    d = s.float() * g
    a = torch.where(d < TINY, 0.0, xb.abs() / torch.where(d < TINY, 1.0, d))
    code = sum((a > t).to(torch.uint8) for t in (0.25, 1.25, 2.5, 5.0)) + sum((a >= t).to(torch.uint8) for t in (0.75, 1.75, 3.5))
    code = (code + (xb < 0).to(torch.uint8) * 8).view(M, K // 2, 2)
    q = code[..., 0] | (code[..., 1] << 4)
    Mp = rup(M, 128)
    sw = F.pad(s.view(M, K // 16).view(torch.uint8), (0, 0, 0, Mp - M)).view(Mp // 128, 4, 32, K // 64, 4)
    return q, sw.permute(0, 3, 2, 1, 4).reshape(-1)


@torch.library.custom_op("nvfp4::gemm", mutates_args=())
def nvfp4_gemm(xq: torch.Tensor, xs: torch.Tensor, wq: torch.Tensor, ws: torch.Tensor) -> torch.Tensor:
    return torch._scaled_mm(xq.view(F4), wq.t(), xs.view(E4), ws, out_dtype=torch.bfloat16)


@nvfp4_gemm.register_fake
def _(xq, xs, wq, ws):
    return xq.new_empty(xq.shape[0], wq.shape[0], dtype=torch.bfloat16)


@torch.library.custom_op("nvfp4::mm", mutates_args=())
def nvfp4_mm(x: torch.Tensor, ga: torch.Tensor, wq: torch.Tensor, ws: torch.Tensor) -> torch.Tensor:
    xq, xs = quant_nvfp4(x, ga)
    return torch._scaled_mm(xq, wq.t(), xs, ws, out_dtype=torch.bfloat16)


@nvfp4_mm.register_fake
def _(x, ga, wq, ws):
    return x.new_empty(x.shape[0], wq.shape[0])


class F4Linear(torch.nn.Module):
    """NVFP4 weight (quantized once) x NVFP4 activation (per-16 scales at run time, fixed global scale)."""

    def __init__(self, lin, weight_amax, input_amax):
        super().__init__()
        w = lin.weight.data
        dev = w.device
        self.gw = torch.tensor(float(weight_amax) / (6 * 448), device=dev, dtype=torch.float32)
        self.ga = torch.tensor(float(input_amax) / (6 * 448), device=dev, dtype=torch.float32)
        self.wq, self.ws = quant_nvfp4(w.contiguous(), self.gw)
        self.alpha = self.ga * self.gw
        self.bias = lin.bias.data if lin.bias is not None else None
        self.out_features = w.shape[0]

    def forward(self, x):
        s = x.shape
        x = x.reshape(-1, s[-1]).to(torch.bfloat16).contiguous()
        if torch.compiler.is_compiling():  # Inductor fuses the quantizer into the op producing x (measured faster)
            y = nvfp4_gemm(*quant_nvfp4_torch(x, self.ga), self.wq, self.ws)
        else:
            y = nvfp4_mm(x, self.ga, self.wq, self.ws)
        y = y * self.alpha.to(y.dtype)
        if self.bias is not None:
            y = y + self.bias
        return y.view(*s[:-1], -1)


class F8Linear(torch.nn.Module):
    """per-tensor FP8 e4m3 weight and activation with the QAD scales."""

    def __init__(self, lin, weight_amax, input_amax):
        super().__init__()
        w = lin.weight.data
        dev = w.device
        self.sw = torch.tensor(float(weight_amax) / 448, device=dev, dtype=torch.float32)
        self.sa = torch.tensor(float(input_amax) / 448, device=dev, dtype=torch.float32)
        self.w8 = (w.float() / self.sw).clamp(-448, 448).to(E4)  # (N, K) row-major -> .t() is column-major as _scaled_mm wants
        self.bias = lin.bias.data.to(torch.bfloat16) if lin.bias is not None else None
        self.out_features = w.shape[0]

    def forward(self, x):
        s = x.shape
        x8 = (x.reshape(-1, s[-1]).float() / self.sa).clamp(-448, 448).to(E4)
        y = torch._scaled_mm(x8, self.w8.t(), scale_a=self.sa, scale_b=self.sw, bias=self.bias, out_dtype=torch.bfloat16)
        return y.view(*s[:-1], -1)


def _selfcheck():
    """the torch and Triton quantizers agree bit for bit; a GEMM through them matches the dequantized operands."""
    torch.manual_seed(0)
    for M, K, N in [(70, 192, 96), (1750, 2560, 512)]:
        x = torch.randn(M, K, device="cuda", dtype=torch.bfloat16) * 3
        x[:, :16] = 1e-7                                                       # a zero block (decoded scale < 1e-5)
        g = (x.abs().amax().float() / (6 * 448)).reshape(())
        q, s = quant_nvfp4(x, g)
        tq, ts = quant_nvfp4_torch(x, g)
        assert torch.equal(tq, q.view(torch.uint8)) and torch.equal(ts, s.view(torch.uint8)), "torch quant != triton"
        assert (q.view(torch.uint8)[:, :8] == 0).all(), "tiny block not zero"
        lin = torch.nn.Linear(K, N, bias=True, device="cuda", dtype=torch.bfloat16)
        f4 = F4Linear(lin, lin.weight.abs().amax(), x.abs().amax())
        f8 = F8Linear(lin, lin.weight.abs().amax(), x.abs().amax())
        ref = torch.nn.functional.linear(x.float(), lin.weight.float(), lin.bias.float())
        e4 = ((f4(x).float() - ref).norm() / ref.norm()).item()
        e8 = ((f8(x).float() - ref).norm() / ref.norm()).item()
        assert e4 < 0.15 and e8 < 0.05, (e4, e8)
        print(f"M={M} K={K} N={N}: rel err nvfp4 {e4:.3f} fp8 {e8:.4f}")
    print("nvfp4 self-check ok")


if __name__ == "__main__":
    _selfcheck()
