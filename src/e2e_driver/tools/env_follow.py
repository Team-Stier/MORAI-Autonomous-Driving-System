#!/usr/bin/env python3
"""Competition weather and time of day per section, for evaluation drives (run on the WSL host with a grpc venv).

Rulebook v1.2 §2-7-2 (10/19 text): weather Sunny or Foggy (Foggy density 1, distance 0) and time 11, 13 or 15 h are
applied at random "by vehicle position and section", at the same positions for every team; the positions and the
probabilities are not published (MORAI, 08/05). Here the section is section_annotator's checkpoint section (1-16):
whenever it changes and holds for HOLD_S, the (weather, hour) for (seed, section, nth entry into it) is sent over the
simulator's Environment gRPC service, so one seed gives the same weather at the same places on every run.
Follows sec_NAME.csv (two_pc.sh's rostopic echo of /section_annotation/state) and logs "wall_t WEATHER HOUR" lines, the
collection's env-*.log format (Alpamayo-Cosmos tools/ledger.py env_table).
  env_follow.py HOST:PORT SEC_CSV LOG SEED STOP_FILE      follow a drive
  env_follow.py HOST:PORT --resend LOG                     send the last logged (weather, hour) again (after a reload)
  env_follow.py HOST:PORT --set WEATHER HOUR               one fixed setting (SUNNY|FOGGY, 11|13|15)
"""
import os, random, sys, time

import grpc

COMBOS = [(w, h) for w in ("SUNNY", "FOGGY") for h in (11, 13, 15)]
WEATHER = {"SUNNY": 0, "CLOUDY": 1, "FOGGY": 2}
HOLD_S = 2.0


def varint(n):
    out = b""
    while True:
        b = n & 0x7F; n >>= 7
        if n:
            out += bytes([b | 0x80])
        else:
            return out + bytes([b])


def send(host, weather, hour):
    ch = grpc.insecure_channel(host)
    call = lambda m, req: ch.unary_unary("/morai_sim_api.environment.Environment/" + m, request_serializer=lambda x: x,
                                         response_deserializer=lambda x: x)(req, timeout=10)
    call("SetWeather", b"\x08" + varint(WEATHER[weather])); call("SetTime", b"\x08" + varint(int(hour)))


def combo(seed, section, visit):
    return random.Random("%s-%d-%d" % (seed, section, visit)).choice(COMBOS)


def follow(host, sec_csv, log, seed, stop):
    while not os.path.exists(sec_csv) and not os.path.exists(stop):
        time.sleep(0.2)
    f, visits, cur, cand, since = open(sec_csv), {}, None, None, 0.0
    w, h = combo(seed, 0, 0)                                # before the first checkpoint: section 0 (UNKNOWN)
    send(host, w, h); open(log, "a").write("%.3f %s %d\n" % (time.time(), w, h))
    while not os.path.exists(stop):
        line = f.readline()
        if not line:
            time.sleep(0.05); continue
        try:
            s = int(line.strip().split(",")[1])
        except (IndexError, ValueError):
            continue                                        # header or a partial line
        if s == cur:
            cand = None; continue
        if s != cand:
            cand, since = s, time.time(); continue
        if time.time() - since >= HOLD_S:                   # a new section held long enough (not a pose glitch)
            cur, cand = s, None
            visits[s] = visits.get(s, -1) + 1
            w, h = combo(seed, s, visits[s])
            send(host, w, h); open(log, "a").write("%.3f %s %d\n" % (time.time(), w, h))


def main():
    a = sys.argv[1:]
    if a[1] == "--set":
        send(a[0], a[2], a[3]); return
    if a[1] == "--resend":
        lines = open(a[2]).read().split() if os.path.exists(a[2]) else []
        if len(lines) >= 3:
            send(a[0], lines[-2], lines[-1])
        return
    follow(a[0], a[1], a[2], a[3], a[4])


assert combo(7, 3, 0) == combo(7, 3, 0) and len({combo(s, 3, 0) for s in range(40)}) == len(COMBOS)

if __name__ == "__main__":
    main()
