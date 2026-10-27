#!/usr/bin/env python3
"""udp_tee.py PORT DST_PORT ...: every datagram arriving on PORT is sent on to 127.0.0.1:DST_PORT for each DST_PORT.
MORAI sends Object Info to one port; two sockets bound to it (SO_REUSEADDR) split the packets between them, so the
shadow's gt-objects-filter-node and rl_episode's critic/replay receiver each get a full copy from here (shadow.sh)."""
import socket
import sys

port, dsts = int(sys.argv[1]), [("127.0.0.1", int(p)) for p in sys.argv[2:]]
rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
rx.bind(("0.0.0.0", port))
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
n = 0
while True:
    pkt = rx.recv(65535)
    for d in dsts:
        tx.sendto(pkt, d)
    n += 1
    if n in (1, 1000) or n % 100000 == 0:
        print(f"{n} datagrams from {port} to {[p for _, p in dsts]}", flush=True)
