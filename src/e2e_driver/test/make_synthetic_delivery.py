"""A full-size stand-in delivery for engine latency work before the real one exists: the real student architecture
(Qwen3-VL-4B + LiDAR ViT copy + the release's expert, StudentVLA from the release config) with random BF16 weights,
written like qad_export.py deliver (final/ in 3 safetensors shards + index, quant_manifest.json), with the layer
formats and amax of the H100 smoke QAD run (train/logs/student/h100_smoke/ssmoke/qad/quant_manifest.json: 600 layers,
240 NVFP4 / 360 FP8). Built on the meta device and written shard by shard, so RAM holds one shard (~4.5 GB).

  winpc (cwd src/e2e_driver/server): HF_HUB_OFFLINE=1 ~/claude-bench/venv-teacher/bin/python ../test/make_synthetic_delivery.py \
      <out dir> <Alpamayo-1.5 release dir> <smoke quant_manifest.json>
"""
import json
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "server"), str(HERE.parent / "server" / "vendor")]

import torch  # noqa: E402
from accelerate import init_empty_weights  # noqa: E402
from safetensors.torch import save_file  # noqa: E402

from morai_vla.model import load_config  # noqa: E402
from morai_vla.student import StudentVLA, _teacher_vit_hidden  # noqa: E402

SHARD_BYTES = 4.8e9


def main():
    out, release, manifest = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
    final = out / "final"
    final.mkdir(parents=True, exist_ok=True)
    extra = {"teacher_vit_hidden": _teacher_vit_hidden(release, "Qwen/Qwen3-VL-8B-Instruct")}
    config = load_config(release, "Qwen/Qwen3-VL-4B-Instruct", "sdpa", {}, None, extra)
    torch.set_default_dtype(torch.bfloat16)
    with init_empty_weights():
        model = StudentVLA(config)
    torch.set_default_dtype(torch.float32)
    sd = model.state_dict()
    if model.vlm.config.tie_word_embeddings:
        sd.pop("vlm.lm_head.weight")                                         # tied to the embedding, as save_pretrained does
    torch.manual_seed(0)
    shards, cur, size, weight_map = [], {}, 0, {}

    def flush():
        nonlocal cur, size
        if cur:
            shards.append(cur); name = f"part{len(shards)}"
            save_file(cur, str(final / name), metadata={"format": "pt"})
            for k in cur: weight_map[k] = name
            cur, size = {}, 0
    for k, v in sd.items():
        t = (torch.randn(v.shape) * 0.02).to(torch.bfloat16) if v.is_meta else v.detach().clone()
        cur[k] = t; size += t.numel() * t.element_size()
        if size >= SHARD_BYTES:
            flush(); shards[-1] = None
    flush(); shards[-1] = None
    n = len(shards)
    names = {f"part{i + 1}": f"model-{i + 1:05d}-of-{n:05d}.safetensors" for i in range(n)}
    for old, new in names.items():
        (final / old).rename(final / new)
    json.dump({"metadata": {}, "weight_map": {k: names[v] for k, v in weight_map.items()}},
              open(final / "model.safetensors.index.json", "w"), indent=1)
    config.save_pretrained(final)
    man = json.load(open(manifest))
    own = set(model.state_dict())
    assert all(k + ".weight" in own for k in man["layers"]), "manifest layer missing from the architecture"
    shutil.copy(manifest, out / "quant_manifest.json")
    (out / "CONTENTS.txt").write_text("synthetic: random weights, H100 smoke QAD formats (engine latency only)\n")
    print(json.dumps({"tensors": len(weight_map), "shards": n, "layers": len(man["layers"])}))


if __name__ == "__main__":
    main()
