"""The model server's pinned training code (server/vendor/morai_vla) is what its manifest says, vlp16_bev's lidar_bev.py
is the same file, and, given a training tree (MORAI_VLA_TRAIN or argv[1]), every vendored file is byte-identical to it.
Usage: python3 test_vendor.py [TRAIN_DIR]"""
import hashlib, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
VENDOR = os.path.join(HERE, "..", "server", "vendor")
sha = lambda p: hashlib.sha256(open(p, "rb").read()).hexdigest()


def test_vendor(train=None):
    rows = [l.split() for l in open(os.path.join(VENDOR, "MANIFEST.sha256")) if not l.startswith("#")]
    assert rows and all(sha(os.path.join(VENDOR, f)) == h for h, f in rows), "vendored file differs from its manifest"
    names = sorted(f for f in os.listdir(os.path.join(VENDOR, "morai_vla")) if f.endswith(".py"))
    assert names == sorted(os.path.basename(f) for _, f in rows), "manifest does not list exactly the vendored files"
    bev = os.path.join(HERE, "..", "..", "vlp16_bev", "src", "vlp16_bev", "lidar_bev.py")
    assert sha(bev) == sha(os.path.join(VENDOR, "morai_vla", "lidar_bev.py")), "vlp16_bev lidar_bev.py != training lidar_bev.py"
    if train:
        diff = [n for n in names if sha(os.path.join(train, "morai_vla", n)) != sha(os.path.join(VENDOR, "morai_vla", n))]
        assert not diff, "differs from %s: %s (tools/vendor_sync.sh)" % (train, diff)


if __name__ == "__main__":
    test_vendor(sys.argv[1] if len(sys.argv) > 1 else os.environ.get("MORAI_VLA_TRAIN")); print("ok")
