"""engine.manifest_layers: a delivery that is not qad_export.py deliver's final/ + quant_manifest.json fails with a
DeliveryError naming the cause (CPU only).   cwd src/e2e_driver/server: python ../test/test_delivery_check.py"""
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "server"))
from engine import DeliveryError, manifest_layers  # noqa: E402

GOOD = {"format": "nvfp4", "block_size": 16, "weight_amax": 1.0, "input_amax": 2.0, "shape": [8, 16]}


def delivery(root, layers, files=("final/config.json", "final/model.safetensors")):
    d = Path(tempfile.mkdtemp(dir=root))
    for f in files:
        (d / f).parent.mkdir(parents=True, exist_ok=True); (d / f).write_text("{}")
    if layers is not None:
        json.dump({"meta": {}, "layers": layers}, open(d / "quant_manifest.json", "w"))
    return d


def fails(d, text):
    try:
        manifest_layers(d)
    except DeliveryError as e:
        assert text in str(e), (text, str(e))
        return
    raise AssertionError("no DeliveryError for " + text)


with tempfile.TemporaryDirectory() as root:
    assert manifest_layers(delivery(root, {"a": GOOD, "b": dict(GOOD, format="fp8", block_size=None)}))
    fails(delivery(root, None), "quant_manifest.json")
    fails(delivery(root, {"a": GOOD}, files=("final/config.json",)), "model.safetensors")
    fails(delivery(root, {}), "no 'layers'")
    fails(delivery(root, {"a": dict(GOOD, format="int4")}), "format 'int4'")
    fails(delivery(root, {"a": dict(GOOD, block_size=32)}), "block_size 32")
    fails(delivery(root, {"a": dict(GOOD, input_amax=None)}), "input_amax None")
print("delivery checks ok")
