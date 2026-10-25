"""Messages between the ROS node (python 3.8, numpy 1.17, in the Noetic chroot) and the model server (python 3.12,
numpy 2, outside it, on the GPU). No pickle: numpy 2 pickles do not load in numpy 1.17. A frame is a 4-byte big-endian
header length, a JSON header, then the raw buffers it points to: arrays as {"__nd__": [dtype, shape, i]}, bytes as
{"__b__": i}.
"""
import json
import socket
import struct

import numpy as np


def pack(obj):
    bufs = []

    def enc(o):
        if isinstance(o, np.ndarray):
            o = np.ascontiguousarray(o); bufs.append(o.tobytes())
            return {"__nd__": [o.dtype.str, list(o.shape), len(bufs) - 1]}
        if isinstance(o, (bytes, bytearray)):
            bufs.append(bytes(o)); return {"__b__": len(bufs) - 1}
        if isinstance(o, dict):
            return {k: enc(v) for k, v in o.items()}
        if isinstance(o, (list, tuple)):
            return [enc(v) for v in o]
        if isinstance(o, np.generic):
            return o.item()
        return o

    body = enc(obj)
    head = json.dumps({"obj": body, "sizes": [len(b) for b in bufs]}).encode()
    return struct.pack(">I", len(head)) + head + b"".join(bufs)


def unpack(frame):
    n = struct.unpack(">I", frame[:4])[0]
    head = json.loads(frame[4:4 + n].decode())
    bufs, off = [], 4 + n
    for size in head["sizes"]:
        bufs.append(frame[off:off + size]); off += size

    def dec(o):
        if isinstance(o, dict):
            if "__nd__" in o:
                dt, shape, i = o["__nd__"]
                return np.frombuffer(bufs[i], np.dtype(dt)).reshape(shape).copy()
            if "__b__" in o:
                return bufs[o["__b__"]]
            return {k: dec(v) for k, v in o.items()}
        if isinstance(o, list):
            return [dec(v) for v in o]
        return o

    return dec(head["obj"])


def _recv_exact(sock, n):
    out = bytearray()
    while len(out) < n:
        chunk = sock.recv(min(n - len(out), 1 << 20))
        if not chunk:
            raise ConnectionError("peer closed")
        out += chunk
    return bytes(out)


def send(sock, obj):
    data = pack(obj)
    sock.sendall(struct.pack(">Q", len(data)) + data)


def recv(sock):
    n = struct.unpack(">Q", _recv_exact(sock, 8))[0]
    return unpack(_recv_exact(sock, n))


class Client:
    """blocking request/response client with reconnect."""

    def __init__(self, host, port, timeout=5.0):
        self.addr, self.timeout, self.sock = (host, int(port)), timeout, None

    def call(self, obj):
        if self.sock is None:
            self.sock = socket.create_connection(self.addr, timeout=self.timeout)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            send(self.sock, obj)
            return recv(self.sock)
        except Exception:
            self.close()
            raise

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None
