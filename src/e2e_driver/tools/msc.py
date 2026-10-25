"""Minimal MSC client (port of morai-use windows_msc.ps1) for a Linux launcher, e.g. the bc-250 simulator host.
Usage: python3 msc.py <LinkConnection.txt> status|execute|map|pause|play|load-network|load-sensors|load-scene [option] [timeout_s]
  map R_KR_PR_K-city_2025/2023_Hyundai_Ioniq5     (re)load the map; the vehicle's default Sensor/Network presets apply
  load-scene rs2041_001/62                        scenario + its EgoNetwork/<name>_MN.json, waits paused at the spawn"""
import socket, struct, sys, time

CMD = {"execute": (1, 4), "map": (2, 1), "pause": (2, 2), "play": (2, 3), "load-network": (2, 17), "load-sensors": (2, 18),
       "load-scene": (2, 19)}


def packet(platform, command, option=""):
    v = (option + "$").encode()
    assert len(v) <= 100 and "$" not in option
    b = bytearray(141)
    b[0:12] = b"#SimControl$"; b[12:16] = struct.pack("<i", 103)
    b[36] = platform; b[37:39] = struct.pack("<H", command); b[39:39 + len(v)] = v; b[139:141] = b"\r\n"
    return bytes(b)


def parse(b):
    if len(b) != 145 or b[:11] != b"#SimStatus$" or struct.unpack("<i", b[11:15])[0] != 108 or b[143:145] != b"\r\n":
        raise ValueError("bad MSC frame")
    cmd = struct.unpack("<H", b[40:42])[0]
    opt = "[login redacted]" if b[39] == 1 and cmd == 1 else b[42:142].rstrip(b"\0").decode(errors="replace").rstrip("$")
    return dict(platform=b[35], stage=b[36], status=struct.unpack("<H", b[37:39])[0], command_platform=b[39], command=cmd,
                option=opt, result=b[142])


def main():
    cfg = dict(l.split(":", 1) for l in open(sys.argv[1]).read().splitlines() if ":" in l)
    cfg = {k.strip(): v.strip() for k, v in cfg.items()}
    action = sys.argv[2]; option = sys.argv[3] if len(sys.argv) > 3 else ""
    timeout = float(sys.argv[4]) if len(sys.argv) > 4 else 15
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((cfg["Request Destination IP"], int(cfg["Request Destination PORT"]))); s.settimeout(1)
    dest = (cfg["Receive Host IP"], int(cfg["Receive Host PORT"]))
    end, sent = time.time() + timeout, False
    while time.time() < end:
        try:
            b, _ = s.recvfrom(4096)
        except socket.timeout:
            continue
        st = parse(b)
        if action == "status":
            print(st); return
        if action == "login":                      # option = path of {"username","password"} JSON; run by the user only
            if st["platform"] == 2 or st["stage"] == 2:
                print("logged in (verify the account header)", st); return
            if not sent:
                import json
                c = json.load(open(option)); s.sendto(packet(1, 1, c["username"] + "/" + c["password"]), dest); sent = True; c = None
            elif st["command_platform"] == 1 and st["command"] == 1 and st["result"] not in (0, 1):
                print("login rejected", st); sys.exit(1)
            continue
        p, c = CMD[action]
        if not sent:
            s.sendto(packet(p, c, option), dest); sent = True
        elif action == "execute" and st["platform"] == 2:
            print("ok", st); return
        elif action in ("pause", "play") and st["stage"] == 2 and st["status"] == (2 if action == "pause" else 1):
            print("ok", st); return
        elif st["command_platform"] == p and st["command"] == c and st["result"] not in (0,) and not (st["status"] & 4096):
            if action == "map" and st["result"] == 1 and (st["stage"] != 2 or st["status"] not in (1, 5)):
                continue
            print("ok" if st["result"] == 1 else "fail", st); return
    print("timeout sent=%s" % sent); sys.exit(1)


if __name__ == "__main__":
    t = packet(2, 18, "sensor.json"); assert len(t) == 141 and t[36] == 2 and t[37] == 18 and t[50] == 36   # same self-test as the .ps1
    main()
