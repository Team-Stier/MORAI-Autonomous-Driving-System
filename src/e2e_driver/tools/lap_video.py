#!/usr/bin/env python3
"""Lap video at 1x simulator time from the simulator window (viewer.py's MJPEG on bc-250, VIEW_FPS / VIEW_WIDTH of
eval_manager.py). bc-250 runs below real time, so the recording is put on the estimated simulator clock tau.

    lap_video.py record URL DIR                  frames DIR/NNNNNN.jpg + DIR/index.txt (Mac wall time), until killed
    lap_video.py make DIR CLOCK.json OUT.mp4 [FPS]

make: CLOCK = eval_manager's lapK.clock.json, (wall, tau) at 20 Hz; each frame's wall time -> tau by interpolation;
FPS (30) frames per simulator second, each the newest frame at or before its tick; 1280x720 H.264 (ffmpeg, CRF 23).
"""
import json
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np


def record(url, d):
    d = Path(d); d.mkdir(parents=True, exist_ok=True)
    idx, n, f = open(d / "index.txt", "a", buffering=1), 0, urllib.request.urlopen(url, timeout=30)
    while True:
        line = f.readline()
        if not line:
            break
        if line.lower().startswith(b"content-length:"):
            size = int(line.split(b":")[1])
            f.readline()                                          # blank line before the JPEG
            data, t = f.read(size), time.time()
            (d / f"{n:06d}.jpg").write_bytes(data)
            idx.write(f"{n:06d}.jpg {t:.3f}\n"); n += 1


def make(d, clock, out, fps=30.0):
    d = Path(d)
    names, wall = zip(*[(a, float(b)) for a, b in (l.split() for l in open(d / "index.txt"))])
    c = np.array(json.load(open(clock)))
    wall = np.array(wall)
    keep = (wall >= c[0, 0]) & (wall <= c[-1, 0])                  # frames during the lap
    tau = np.interp(wall[keep], c[:, 0], c[:, 1])
    names = np.array(names)[keep]
    ticks = np.arange(tau[0], tau[-1], 1.0 / fps)
    pick = np.searchsorted(tau, ticks, side="right") - 1          # newest frame at or before each tick
    with open(d / "concat.txt", "w") as f:
        for n in names[pick]:
            f.write(f"file '{(d / n).resolve()}'\nduration {1.0 / fps:.6f}\n")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(d / "concat.txt"),
                    "-vf", "scale=1280:720:force_original_aspect_ratio=decrease,pad=1280:720:(ow-iw)/2:(oh-ih)/2,fps=%g" % fps,
                    "-c:v", "libx264", "-crf", "23", "-preset", "veryfast", "-pix_fmt", "yuv420p", str(out)], check=True)
    wall_span = float(wall[keep][-1] - wall[keep][0])
    print(json.dumps(dict(frames=int(keep.sum()), video_s=round(len(ticks) / fps, 1), tau_s=round(float(tau[-1] - tau[0]), 1),
                          wall_s=round(wall_span, 1), capture_fps_wall=round(keep.sum() / max(wall_span, 1e-9), 1),
                          mb=round(Path(out).stat().st_size / 2**20, 1))))


if __name__ == "__main__":
    if sys.argv[1] == "record":
        record(sys.argv[2], sys.argv[3])
    else:
        make(sys.argv[2], sys.argv[3], sys.argv[4], float(sys.argv[5]) if len(sys.argv) > 5 else 30.0)
