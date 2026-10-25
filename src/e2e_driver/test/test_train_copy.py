"""The ROS side's copies of training code are the training code: same function sources, same constants, same file.
Usage: python3 test_train_copy.py <Alpamayo-Cosmos train dir>   (no third-party imports; also runs under pytest with
MORAI_VLA_TRAIN set)."""
import ast, hashlib, os, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
COPY = HERE.parent / "src/e2e_driver/train_copy.py"
LIDAR_COPY = HERE.parents[1] / "vlp16_bev/src/vlp16_bev/lidar_bev.py"


def defs(path):
    src = Path(path).read_text(); tree = ast.parse(src); out = {}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef):
            out[node.name] = ast.get_source_segment(src, node)
        elif isinstance(node, ast.Assign):
            for t in node.targets:
                names = [e.id for e in t.elts] if isinstance(t, ast.Tuple) else [getattr(t, "id", None)]
                vals = node.value.elts if isinstance(node.value, ast.Tuple) else [node.value] * len(names)
                for n, v in zip(names, vals): out[n] = ast.unparse(v) if hasattr(ast, "unparse") else ast.dump(v)
    return out


def check(train):
    train = Path(train); copy = defs(COPY)
    data, bi = defs(train / "morai_vla/data.py"), defs(train / "morai_vla/build_index.py")
    for name in ("frame_targets", "nearest", "select_distinct", "N_FRAMES", "N_HIST", "N_FUT", "DT", "MAX_FRAME_GAP", "MAX_SCAN_GAP"):
        assert copy[name] == data[name], name
    for name in ("nav_text", "NAV_RANGE"):
        assert copy[name] == bi[name], name
    a, b = (hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in (LIDAR_COPY, train / "morai_vla/lidar_bev.py"))
    assert a == b, "vlp16_bev/lidar_bev.py differs from train/morai_vla/lidar_bev.py"
    return "train_copy.py and lidar_bev.py match %s (lidar_bev sha256 %s)" % (train, a[:12])


def test_train_copy():
    print(check(os.path.expanduser(os.environ.get("MORAI_VLA_TRAIN", "~/morai-student/train"))))


if __name__ == "__main__":
    print(check(sys.argv[1]))
