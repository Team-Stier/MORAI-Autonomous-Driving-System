"""model_server --sims (RL actor) without a GPU (fake backend): three clients at once each get their own answers (id,
one simulator slot each), a fourth is refused while three are connected, a slot frees when its client goes; and
backends.rl_logp against the learner's Gaussian (Alpamayo-Cosmos train/rl/policy.py gauss_logp).

    <training venv>/bin/python test/test_rl_server.py
"""
import math
import socket
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "server"), str(HERE.parent / "src"), str(HERE.parent / "server" / "vendor")]

import torch  # noqa: E402

from backends import rl_logp  # noqa: E402
from e2e_driver import wire  # noqa: E402


def test_logp():
    torch.manual_seed(0)
    sigma = torch.tensor([0.0, 0.3] + [0.05] * 8)
    mu, eps = torch.randn(10, 64, 2), torch.randn(10, 1, 64, 2)
    x = mu + sigma.view(-1, 1, 1) * eps[:, 0]
    s = sigma.clamp_min(1e-30).view(-1, 1, 1)
    ref = (-0.5 * ((x - mu) / s) ** 2 - s.log() - 0.5 * math.log(2 * math.pi)).sum((-1, -2))
    ref[0] = 0.0                                                           # sigma 0: a deterministic step
    got = rl_logp(eps, sigma)
    assert torch.allclose(got, ref, rtol=1e-4, atol=1e-2), (got, ref)
    print("rl_logp ok", got[:3].tolist())


def connect(port):
    c = socket.create_connection(("127.0.0.1", port), timeout=20)
    c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return c


def test_server():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]
    p = subprocess.Popen([sys.executable, str(HERE.parent / "server" / "model_server.py"), "--backend", "fake", "--sims", "3",
                          "--port", str(port)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        for _ in range(100):
            try:
                connect(port).close(); break                               # (that client takes a slot and goes)
            except OSError:
                time.sleep(0.2)
        req = wire.unpack((HERE.parent / "assets" / "warmup" / "req_000600.bin").read_bytes())
        time.sleep(0.5)                                                    # the probe's slot is freed
        conns = [connect(port) for _ in range(3)]
        time.sleep(0.5)
        extra = connect(port)
        try:
            wire.recv(extra); raise AssertionError("a fourth client was served")
        except ConnectionError:
            pass
        for i, c in enumerate(conns):
            for j in range(2):
                wire.send(c, dict(req, id=100 * i + j))
        sims = []
        for i, c in enumerate(conns):
            got = [wire.recv(c) for _ in range(2)]
            assert [g["id"] for g in got] == [100 * i, 100 * i + 1], [g["id"] for g in got]
            assert got[0]["sim"] == got[1]["sim"]
            sims.append(got[0]["sim"])
        assert sorted(sims) == [0, 1, 2], sims
        conns[1].close()
        for _ in range(50):                                                # the freed slot goes to the next client
            c = connect(port)
            try:
                wire.send(c, dict(req, id=7)); r = wire.recv(c); break
            except ConnectionError:
                time.sleep(0.2)
        assert r["id"] == 7 and r["sim"] == sims[1], r
        print("serve_batch ok: slots", sims, "reused", r["sim"])
    finally:
        p.terminate()
        out = p.communicate(timeout=10)[0]
        if sys.exc_info()[0] is not None:
            print(out)


if __name__ == "__main__":
    test_logp()
    test_server()
