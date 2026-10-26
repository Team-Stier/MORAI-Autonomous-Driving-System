#!/usr/bin/env python3
"""Lap video at 1x simulator time from the simulator window (viewer.py's MJPEG on bc-250, VIEW_FPS / VIEW_WIDTH of
eval_manager.py). bc-250 runs below real time, so the recording is put on the estimated simulator clock tau.

    lap_video.py record URL DIR                  frames DIR/NNNNNN.jpg + DIR/index.txt (Mac wall time), until killed
    lap_video.py make DIR CLOCK.json OUT.mp4 [FPS] [TELEMETRY.csv OFFSET_S]
    lap_video.py debug-telemetry DBG.csv STACK.log OUT.csv     (a run without telemetry_rec.py: the node's commands only)

make: CLOCK = eval_manager's lapK.clock.json, (wall, tau) at 20 Hz; each frame's wall time -> tau by interpolation;
FPS (30) frames per simulator second, each the newest frame at or before its tick; 1280x720 H.264 (ffmpeg, CRF 23, LAP_CRF):
the (square) simulator window on the left 720 x 720, a dashboard on the right 560 px (it covers nothing of the car):
speedometer, steering wheel icon + front wheel angle, accel / brake over the last 10 s of simulator time, applied
values (competition_status, solid) and the node's command (/control_cmd, dashed). TELEMETRY = telemetry_rec.py's CSV
on winpc's wall clock; OFFSET_S = Mac wall - winpc wall (0.04 s on 10/27).
"""
import json
import math
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np


def record(url, d):
    """frames until killed; waits for the viewer and reconnects when the stream drops (eval_manager starts the viewer)."""
    d = Path(d); d.mkdir(parents=True, exist_ok=True)
    idx, n = open(d / "index.txt", "a", buffering=1), 0
    while True:
        try:
            f = urllib.request.urlopen(url, timeout=30)
            while True:
                line = f.readline()
                if not line:
                    break
                if line.lower().startswith(b"content-length:"):
                    size = int(line.split(b":")[1])
                    f.readline()                                  # blank line before the JPEG
                    data, t = f.read(size), time.time()
                    (d / f"{n:06d}.jpg").write_bytes(data)
                    idx.write(f"{n:06d}.jpg {t:.3f}\n"); n += 1
        except OSError:
            pass
        time.sleep(1.0)


FONT = "/System/Library/Fonts/Helvetica.ttc"
W, H, SIDE = 1280, 720, 720                                      # output, simulator view (left square), panel from x = SIDE


def font(size):
    from PIL import ImageFont
    try:
        return ImageFont.truetype(FONT, size)
    except OSError:
        return ImageFont.load_default()


def load_telemetry(path, clock, offset):
    """kind -> (tau, columns) on the video's tau: winpc wall + offset = Mac wall -> tau through the lap clock."""
    import pandas as pd
    t = pd.read_csv(path)
    t["tau"] = np.interp(t.wall.to_numpy(float) + offset, clock[:, 0], clock[:, 1], left=np.nan, right=np.nan)
    t = t[np.isfinite(t.tau)]
    return {k: g.sort_values("tau") for k, g in t.groupby("kind")}


def last(g, tau, col):
    if g is None or not len(g):
        return float("nan")
    i = np.searchsorted(g.tau.to_numpy(), tau, side="right") - 1
    return float(g[col].iat[i]) if i >= 0 else float("nan")


def dashboard(img, tel, tau, tau0):
    from PIL import ImageDraw
    dr, x0 = ImageDraw.Draw(img), SIDE
    s, c = tel.get("s"), tel.get("c")
    f_big, f_mid, f_small = font(40), font(22), font(16)
    dr.text((x0 + 20, 12), "t = %6.1f s (simulator)" % (tau - tau0), font=f_mid, fill=(220, 220, 220))
    # speedometer: 0-100 km/h over 240 deg
    spd = last(s, tau, "speed_kph")
    cx, cy, r = x0 + 140, 175, 105
    dr.arc((cx - r, cy - r, cx + r, cy + r), 150, 390, fill=(90, 90, 100), width=10)
    if np.isfinite(spd):
        ang = math.radians(150 + 240 * min(max(spd, 0.0), 100.0) / 100.0)
        dr.arc((cx - r, cy - r, cx + r, cy + r), 150, math.degrees(ang), fill=(80, 170, 255), width=10)
        dr.line((cx, cy, cx + 0.85 * r * math.cos(ang), cy + 0.85 * r * math.sin(ang)), fill=(255, 255, 255), width=4)
    dr.text((cx, cy + 25), "%s" % ("--" if not np.isfinite(spd) else "%.0f" % abs(spd)), font=f_big, fill=(255, 255, 255), anchor="mm")
    dr.text((cx, cy + 62), "km/h", font=f_small, fill=(170, 170, 170), anchor="mm")
    # steering wheel: icon turned by the handwheel angle (front wheel x 11.25: 40 deg lock = 450 deg), left +
    st, stc = last(s, tau, "steer_deg"), last(c, tau, "steer_deg")
    show = st if np.isfinite(st) else stc
    wx, wy, wr = x0 + 420, 175, 80
    dr.ellipse((wx - wr, wy - wr, wx + wr, wy + wr), outline=(220, 220, 220), width=9)
    rot = math.radians(-11.25 * (show if np.isfinite(show) else 0.0))       # image y is down: left (+) turns anticlockwise
    for a in (math.pi / 2, math.pi * 7 / 6, -math.pi / 6):
        dr.line((wx, wy, wx + wr * math.cos(a + rot), wy + wr * math.sin(a + rot)), fill=(220, 220, 220), width=7)
    dr.ellipse((wx - 14, wy - 14, wx + 14, wy + 14), fill=(220, 220, 220))
    dr.text((wx, wy + wr + 28), "steer %s deg" % ("--" if not np.isfinite(show) else "%+.1f" % show), font=f_mid, fill=(255, 255, 255), anchor="mm")
    if np.isfinite(stc):
        dr.text((wx, wy + wr + 54), "cmd %+.1f deg" % stc, font=f_small, fill=(170, 170, 170), anchor="mm")
    # accel / brake over the last 10 s: applied solid, command dashed
    gx0, gx1, gy0, gy1 = x0 + 30, W - 20, 400, 690
    dr.rectangle((gx0, gy0, gx1, gy1), outline=(90, 90, 100))
    for q in (0.25, 0.5, 0.75):
        y = gy1 - q * (gy1 - gy0); dr.line((gx0, y, gx1, y), fill=(45, 45, 52))
    dr.text((gx0, gy0 - 30), "accel / brake, last 10 s", font=f_mid, fill=(220, 220, 220))
    dr.text((gx1, gy0 - 28), "solid applied, dashed cmd", font=f_small, fill=(150, 150, 150), anchor="ra")
    X = lambda t_: gx0 + (t_ - (tau - 10.0)) / 10.0 * (gx1 - gx0)
    Y = lambda v_: gy1 - min(max(v_, 0.0), 1.0) * (gy1 - gy0)
    for g, dashed in ((s, False), (c, True)):
        if g is None:
            continue
        w = g[(g.tau >= tau - 10.0) & (g.tau <= tau)]
        for col, color in (("accel", (90, 210, 110)), ("brake", (240, 80, 80))):
            pts = [(X(a), Y(b)) for a, b in zip(w.tau, w[col]) if np.isfinite(b)]
            for k, (p0, p1) in enumerate(zip(pts, pts[1:])):
                if not dashed or (k // 3) % 2 == 0:
                    dr.line((p0, p1), fill=color, width=2 if dashed else 3)
    for col, color, yy in (("accel", (90, 210, 110), gy0 + 8), ("brake", (240, 80, 80), gy0 + 30)):
        v_ = last(s, tau, col); vc = last(c, tau, col)
        dr.text((gx0 + 10, yy), "%s %s  (cmd %s)" % (col, "--" if not np.isfinite(v_) else "%.2f" % v_, "--" if not np.isfinite(vc) else "%.2f" % vc),
                font=f_small, fill=color)


def debug_telemetry(dbg, stack_log, out):
    """telemetry rows from the node's debug topic (rostopic echo -p CSV) and its stack.log: tau of the stack -> winpc wall
    from the log's [wall, ros time] pairs; speed = the node's status speed; command = control.py on the reply's (a,
    kappa, hold) at the measured speed, as published on the reply (applied values were not recorded)."""
    import re
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from e2e_driver.control import Controller
    pairs = np.array([(float(a), float(b)) for a, b in re.findall(r"\[(\d{10}\.\d+), (\d{10}\.\d+)\]", open(stack_log, errors="ignore").read())])
    pairs = pairs[np.argsort(pairs[:, 1])]
    ctl = Controller()
    with open(out, "w") as f:
        f.write("wall,kind,speed_kph,steer_deg,accel,brake\n")
        for line in open(dbg).read().splitlines()[1:]:
            r = json.loads(line.split(",", 1)[1]); cmd = r.get("cmd") or {}
            wall = float(np.interp(r["t0"], pairs[:, 1], pairs[:, 0]))
            acc, brk = ctl.pedals(cmd.get("a", 0.0), r["v"])
            if cmd.get("hold"):
                acc, brk = 0.0, max(brk, ctl.p["hold_brake"])
            steer = math.degrees(math.atan(ctl.p["wheelbase_m"] * cmd.get("kappa", 0.0) / ctl.p["steer_gain"]))
            f.write("%.3f,s,%.2f,,,\n%.3f,c,,%.2f,%.3f,%.3f\n" % (wall, 3.6 * r["v"], wall, max(min(steer, 40.0), -40.0), acc, brk))


def make(d, clock, out, fps=30.0, telemetry=None, offset=0.0):
    from PIL import Image
    d = Path(d)
    names, wall = zip(*[(a, float(b)) for a, b in (l.split() for l in open(d / "index.txt"))])
    c = np.array(json.load(open(clock)))
    wall = np.array(wall)
    keep = (wall >= c[0, 0]) & (wall <= c[-1, 0])                  # frames during the lap
    tau = np.interp(wall[keep], c[:, 0], c[:, 1])
    names = np.array(names)[keep]
    ticks = np.arange(tau[0], tau[-1], 1.0 / fps)
    pick = np.searchsorted(tau, ticks, side="right") - 1          # newest frame at or before each tick
    tel = load_telemetry(telemetry, c, offset) if telemetry else {}
    import os
    first = Image.open(d / names[0])                               # no telemetry: the whole view, SIDE high (W1: 1280)
    vw = SIDE if tel else 2 * round(first.width * SIDE / first.height / 2)
    size = f"{W}x{H}" if tel else f"{vw}x{SIDE}"
    enc = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", size,
                            "-r", "%g" % fps, "-i", "-", "-c:v", "libx264", "-crf", os.environ.get("LAP_CRF", "23"), "-preset", "veryfast",
                            "-pix_fmt", "yuv420p", str(out)], stdin=subprocess.PIPE)
    view, last_name = None, None
    for tick, n in zip(ticks, names[pick]):
        if n != last_name:
            im = Image.open(d / n).convert("RGB")
            view, last_name = im.resize((round(im.width * SIDE / im.height), SIDE)), n
        if tel:
            canvas = Image.new("RGB", (W, H), (18, 18, 22))
            canvas.paste(view.crop((0, 0, min(view.width, SIDE), SIDE)), (0, 0))
            dashboard(canvas, tel, float(tick), float(ticks[0]))
        else:                                                  # no telemetry: the simulator view alone, vw x SIDE
            canvas = view.crop((0, 0, vw, SIDE))
        enc.stdin.write(canvas.tobytes())
    enc.stdin.close()
    if enc.wait():
        raise RuntimeError("ffmpeg failed")
    wall_span = float(wall[keep][-1] - wall[keep][0])
    print(json.dumps(dict(frames=int(keep.sum()), video_s=round(len(ticks) / fps, 1), tau_s=round(float(tau[-1] - tau[0]), 1),
                          wall_s=round(wall_span, 1), capture_fps_wall=round(keep.sum() / max(wall_span, 1e-9), 1),
                          mb=round(Path(out).stat().st_size / 2**20, 1))))


if __name__ == "__main__":
    if sys.argv[1] == "record":
        record(sys.argv[2], sys.argv[3])
    elif sys.argv[1] == "debug-telemetry":
        debug_telemetry(sys.argv[2], sys.argv[3], sys.argv[4])
    else:
        a = sys.argv[2:]
        make(a[0], a[1], a[2], float(a[3]) if len(a) > 3 else 30.0, a[4] if len(a) > 4 else None, float(a[5]) if len(a) > 5 else 0.0)
