"""Raw Ego status (9095, judge-only: measurement only) log: wall time, payload hash, speed, x, y per packet until the stop
file exists. Sim-side stalls show as gaps, scene reloads as jumps. Usage: python3 gtraw.py OUT.txt STOP_FILE"""
import hashlib, os, socket, struct, sys, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", 9095)); s.settimeout(1)
out = open(sys.argv[1], "w")
while not os.path.exists(sys.argv[2]):
    try:
        b, _ = s.recvfrom(65536)
    except socket.timeout:
        continue
    x, y = struct.unpack_from("<2f", b, 77)
    out.write("%.4f %s %.3f %.3f %.3f\n" % (time.time(), hashlib.md5(b).hexdigest()[:8], struct.unpack_from("<f", b, 37)[0], x, y))
