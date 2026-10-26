"""A stand-in quant_manifest.json at a QAD bit budget (e.g. 4.75 effective bits: ~93 % NVFP4, ~7 % FP8 of the LLM +
expert weights), for testing the engine on the final delivery's format before it exists. Every LLM decoder and expert
linear starts NVFP4; whole blocks then turn FP8 from both ends inward, alternating LLM and expert (LLM first, expert
first, LLM last, expert last, ...) until the weight-weighted bits (morai_vla.qad.effective_bits: NVFP4 4.5, FP8 8) are
closest to the budget (4.75: 5 blocks, 35 of 504 linears, 4.754 bits).
The real delivery's FP8 layers are chosen by AutoQuantize sensitivity; the engine reads whatever the manifest says.

  python budget_manifest.py <a manifest listing every layer with amax> <bits> <out quant_manifest.json>
"""
import itertools
import json
import re
import sys

BITS = {"nvfp4": 4.5, "fp8": 8.0}
PARTS = ("language_model.layers.", "expert.layers.")


def bits(layers):
    rows = [(v["shape"][0] * v["shape"][1], BITS[v["format"]]) for k, v in layers.items() if any(p in k for p in PARTS)]
    return sum(a * b for a, b in rows) / sum(a for a, _ in rows)


def main():
    src, target, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
    man = json.load(open(src))
    layers = man["layers"]
    blocks = {}
    for k, v in layers.items():
        for p in PARTS:
            m = re.search(re.escape(p) + r"(\d+)\.", k)
            if m:
                v.update(format="nvfp4", block_size=16)
                blocks.setdefault((p, int(m.group(1))), []).append(k)
                break
    order = []                                                     # per part: first, last, second, second last, ...
    for p in PARTS:
        idx = sorted(i for q, i in blocks if q == p)
        order.append([(p, i) for i in dict.fromkeys(itertools.chain.from_iterable(zip(idx, reversed(idx))))])
    fp8 = []
    for blk in (b for b in itertools.chain.from_iterable(itertools.zip_longest(*order)) if b):
        before = bits(layers)
        for k in blocks[blk]: layers[k].update(format="fp8", block_size=None)
        if abs(bits(layers) - target) > abs(before - target):
            for k in blocks[blk]: layers[k].update(format="nvfp4", block_size=16)
            break
        fp8.append(blk)
    count = {f: sum(v["format"] == f for v in layers.values()) for f in BITS}
    man["meta"] = dict(man.get("meta", {}), count=count, effective_bits_searched=round(bits(layers), 3),
                       note="stand-in: FP8 blocks %s (budget_manifest.py %s)" % (fp8, target))
    json.dump(man, open(out, "w"), indent=1)
    print(json.dumps(man["meta"]))


if __name__ == "__main__":
    main()
