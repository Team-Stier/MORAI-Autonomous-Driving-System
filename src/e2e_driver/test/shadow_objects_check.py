"""shadow_objects.load_visible (the label checker's visible() lifted from its source for the ROS python) gives the same
answer as label_check.visible on recorded shadow frames (shadow_rec obj rows with the GT ego pose of their time), and
loads without pandas (run it twice: the second run without pandas checks the loader alone).
Mac: uv run --no-project --with numpy,pyyaml,pandas python shadow_objects_check.py SCORER_DIR REC_DIR [N]
     uv run --no-project --with numpy,pyyaml python shadow_objects_check.py SCORER_DIR REC_DIR [N]
"""
import gzip
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tools" / "pod"))
from shadow_objects import load_visible  # noqa: E402

scorer, rec, n = sys.argv[1], Path(sys.argv[2]), int(sys.argv[3]) if len(sys.argv) > 3 else 2000
vis = load_visible(scorer)
try:
    import pandas  # noqa: F401
    sys.path.insert(0, scorer)
    import yaml
    from label_check import visible
    cfg = yaml.safe_load((Path(scorer) / "label_check.yaml").read_text())["visibility"]
except ImportError:
    visible = None
ego, frames = [], []
for f in sorted(rec.glob("rec_*.jsonl*")):
    for line in (gzip.open(f, "rt") if f.suffix == ".gz" else open(f)):
        if line.startswith('["ego"'):
            ego.append(json.loads(line)[1:5])
        elif line.startswith('["obj"'):
            frames.append(line)
ego = np.array(ego)
frames = [json.loads(frames[i]) for i in np.linspace(0, len(frames) - 1, min(n, len(frames))).astype(int)]   # the whole drive
seen_n = tot = 0
for _, t, objs in frames:
    i = min(np.searchsorted(ego[:, 0], t), len(ego) - 1)
    x, y, yaw = ego[i, 1:4]
    o = [(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[8], r[9]) for r in objs]
    a = vis(x, y, yaw, o)
    if visible is not None:
        assert np.array_equal(a, visible(x, y, yaw, o, cfg)), t
    seen_n, tot = seen_n + int(a.sum()), tot + len(o)
print(f"ok: {len(frames)} frames, {tot} objects, {seen_n} visible ({seen_n / max(tot, 1):.0%}),"
      f" {'same as label_check.visible' if visible else 'loader only (no pandas)'}")
