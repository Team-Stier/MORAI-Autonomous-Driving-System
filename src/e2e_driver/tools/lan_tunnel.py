#!/usr/bin/env python3
"""UDP-over-TCP tunnel for the two-PC setup: MORAI on one host (sim role), the ROS stack on another (stack role).

The simulator keeps its collection presets (every output to 127.0.0.1 standard ports, control in on 127.0.0.1:9093). The
sim role binds those UDP ports on the simulator host's loopback and frames each datagram unchanged as
[u16 port][u32 length][bytes] onto one TCP connection (the framing of the collection tunnel, morai-parallel-collect
wsl/udp-tunnel.py and win/udp_tunnel.ps1). The stack role re-sends them from 127.0.0.1 to 127.0.0.1:<port> on the stack
host (Object Info 7505 and Ego status 9095 to 127.255.255.255, so several receivers get them) and frames the bridge's
control datagrams (sent to 127.0.0.1:9093 there) back to the simulator host's 127.0.0.1:9093.
The sim role listens and the stack role connects, so only the simulator host takes an inbound TCP connection (WSL on
winpc cannot accept LAN connections without Windows port forwarding). TCP keeps 65 KB camera datagrams whole.

  simulator host:  python3 lan_tunnel.py sim   --listen 0.0.0.0:30011 --ports 9291,9293,9295,9297,9301,9999,9099,2368,9095
  stack host:      python3 lan_tunnel.py stack --connect 192.168.0.15:30011
"""
import argparse, selectors, socket, struct, sys, time

BROADCAST = {7505, 9095}
CTRL = 9093


def frames(buf):
    out = []
    while len(buf) >= 6:
        port, size = struct.unpack_from("<HI", buf, 0)
        if len(buf) < 6 + size:
            break
        out.append((port, bytes(buf[6:6 + size]))); buf = buf[6 + size:]
    return out, buf


def frame(port, data):
    return struct.pack("<HI", port, len(data)) + data


def pump(conn, udp_in, deliver):
    """forward datagrams from udp_in sockets onto conn, and frames from conn to deliver(port, data), until conn drops."""
    sel = selectors.DefaultSelector()
    sel.register(conn, selectors.EVENT_READ)
    for s in udp_in: sel.register(s, selectors.EVENT_READ)
    buf = b""
    while True:
        for key, _ in sel.select():
            s = key.fileobj
            if s is conn:
                data = conn.recv(1 << 20)
                if not data:
                    raise ConnectionError("closed")
                got, buf = frames(buf + data)
                for port, d in got: deliver(port, d)
            else:
                while True:
                    try:
                        d = s.recv(65536)
                    except BlockingIOError:
                        break
                    conn.sendall(frame(s.getsockname()[1], d))


def udp_bind(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
    s.bind(("127.0.0.1", port)); s.setblocking(False)
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("role", choices=["sim", "stack"])
    ap.add_argument("--listen", default="0.0.0.0:30011"); ap.add_argument("--connect")
    ap.add_argument("--ports", default="9291,9293,9295,9297,9301,9999,9099,2368,9095,7505")   # 7505: Object Info (objinfo_rec.py)
    a = ap.parse_args()
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1); out.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8 << 20)
    out.bind(("127.0.0.1", 0))
    if a.role == "sim":
        udp_in = [udp_bind(int(p)) for p in a.ports.split(",")]
        deliver = lambda port, d: out.sendto(d, ("127.0.0.1", port))                 # control back into the simulator
        host, port = a.listen.rsplit(":", 1)
        srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); srv.bind((host, int(port))); srv.listen(1)
        print("sim tunnel: udp %s -> tcp %s" % (a.ports, a.listen), flush=True)
        while True:
            conn, peer = srv.accept(); conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("connected", peer, flush=True)
            for s in udp_in:                                                          # drop what queued while unconnected
                try:
                    while s.recv(65536): pass
                except BlockingIOError:
                    pass
            try:
                pump(conn, udp_in, deliver)
            except (ConnectionError, OSError) as e:
                print("disconnected", e, flush=True)
            conn.close()
    else:
        udp_in = [udp_bind(CTRL)]                                                     # the bridge's control datagrams
        deliver = lambda port, d: out.sendto(d, ("127.255.255.255" if port in BROADCAST else "127.0.0.1", port))
        host, port = a.connect.rsplit(":", 1)
        print("stack tunnel: tcp %s -> udp 127.0.0.1:<port>, control %d back" % (a.connect, CTRL), flush=True)
        while True:
            try:
                conn = socket.create_connection((host, int(port)), timeout=5); conn.settimeout(None)
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print("connected", a.connect, flush=True)
                pump(conn, udp_in, deliver)
            except (ConnectionError, OSError) as e:
                print("retry:", e, flush=True); time.sleep(1.0)


assert frames(frame(9291, b"abc") + struct.pack("<HI", 1, 5) + b"xy") == ([(9291, b"abc")], struct.pack("<HI", 1, 5) + b"xy")

if __name__ == "__main__":
    main()
