#!/usr/bin/env python3
"""MORAI Object Info (UDP 7505, broadcast on the stack host by lan_tunnel.py) to CSV with this host's wall clock: checks
that NPCs, pedestrians and obstacles of an evaluation scene are there and move. Plain python3, no ROS.
Format as src/visualizer/scripts/object_info_udp.py: 20 records of (id, kind, x, y, z, heading, length, width, height);
kind 0 pedestrian, 1 vehicle, 2 obstacle.        objinfo_rec.py OUT.csv
"""
import math
import socket
import struct
import sys
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("", 7505))
out = open(sys.argv[1], "a", buffering=1)
out.write("wall,id,kind,x,y,heading\n")
while True:
    p, _ = s.recvfrom(4096)
    if len(p) not in (1400, 2160) or p[:14] != b"#MoraiObjInfo$":
        continue
    rs, t = (len(p) - 40) // 20, time.time()
    for i in range(20):
        v = struct.unpack_from("<hh7f", p, 38 + i * rs)
        if v[0] and all(math.isfinite(x) for x in v[2:]):
            out.write("%.3f,%d,%d,%.2f,%.2f,%.1f\n" % (t, v[0], v[1], v[2], v[3], v[5]))
