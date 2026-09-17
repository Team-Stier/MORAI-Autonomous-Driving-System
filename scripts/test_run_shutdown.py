#!/usr/bin/env python3
"""Exercise run.sh cleanup on Linux without starting ROS or touching the network."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def running(pid):
    try:
        return Path("/proc/%s/stat" % pid).read_text().split(") ", 1)[1][0] != "Z"
    except FileNotFoundError:
        return False


def check(signum):
    source = (Path(__file__).resolve().parents[1] / "run.sh").read_text()
    assert source.index("setsid roslaunch velodyne_pointcloud") < source.index("if ! \"$velodyne_ready\"")
    assert source.index("if ! \"$velodyne_ready\"") < source.index("setsid bash src/MORAI-UDP-Bridge-Node/launch.sh")
    cleanup = source[source.index("children=()"):source.index("\nsetsid roslaunch")]
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        worker = root / "worker.py"
        worker.write_text(
            "import os, signal, subprocess, sys, time\n"
            "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
            "child = subprocess.Popen(['sleep', '60'])\n"
            "with open(sys.argv[1], 'w') as f:\n"
            "    f.write(str(os.getpid()) + ' ' + str(child.pid))\n"
            "time.sleep(60)\n"
        )
        script = cleanup + "\n"
        for i in range(2):
            script += 'setsid python3 "%s" "%s" &\nchildren+=("$!")\n' % (worker, root / str(i))
        script += 'wait -n "${children[@]}"\n'
        parent = subprocess.Popen(["bash", "-c", script], start_new_session=True)
        groups = []
        try:
            deadline = time.monotonic() + 5
            while not all((root / str(i)).exists() for i in range(2)):
                assert time.monotonic() < deadline, "workers failed to start"
                time.sleep(0.02)
            pids = [int(pid) for i in range(2) for pid in (root / str(i)).read_text().split()]
            groups = pids[::2]
            parent.send_signal(signum)
            assert parent.wait(timeout=5) == 128 + signum
            deadline = time.monotonic() + 1
            while any(running(pid) for pid in pids) and time.monotonic() < deadline:
                time.sleep(0.02)
            assert not any(running(pid) for pid in pids), "child/grandchild survived cleanup"
        finally:
            for group in groups:
                try:
                    os.killpg(group, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            if parent.poll() is None:
                parent.kill()
                parent.wait()


if __name__ == "__main__":
    check(signal.SIGINT)
    check(signal.SIGTERM)
    print("PASS: SIGINT/SIGTERM remove both process trees, including TERM-ignoring grandchildren")
