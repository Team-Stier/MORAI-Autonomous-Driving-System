#!/usr/bin/env python3
"""Drive MORAI's official Time / Weather Changer (WeatherTimeChanger_Linux, distributed 9/28) from scripts, on the
simulator host. The program is a Tk GUI without a command line (guide 2 p.12: Connect, then the Time and Weather
buttons), so it runs in a private nested X server (Xephyr DISPLAY :7, a small window on the host desktop) and its
buttons are pressed with XTEST. System python3 with python-xlib (Pillow for `shot`).

    wtc.py up PROGRAM          Xephyr + the program, then Connect (the program sends Simulation.Start in real-time
                               mode, which a running simulator answers with 'same map')
    wtc.py set WEATHER HOUR    press Sunny|Foggy and 11|13|15:00
    wtc.py down | tree | shot OUT.png

Widgets are found by creation order: Tk stacks siblings in creation order, and weather_time_gui._build_ui creates the
Connection (Address, entry, Port, entry, Connect, Disconnect, status), Time (11:00, 13:00, 15:00, current, interval
label, entry, Start Auto), Weather (Sunny, Foggy, current, interval label, entry, Start Auto) and Log frames.
"""
import os
import subprocess
import sys
import time

from Xlib import X, display
from Xlib.ext import xtest

DISP = ":7"
PIDS = os.path.expanduser("~/morai-eval-wtc.pids")
HOURS, WEATHERS = {11: 0, 13: 1, 15: 2}, {"SUNNY": 0, "FOGGY": 1}


def frames(d):
    """the toplevel's four frames: the window whose mapped children hold 7, 7, 6 and >= 1 widgets."""
    def kids(w):
        return [c for c in w.query_tree().children if c.get_attributes().map_state == X.IsViewable]

    def walk(w):
        k = kids(w)
        if len(k) == 4 and [len(kids(c)) for c in k[:3]] == [7, 7, 6]:
            return [kids(c) for c in k]
        for c in k:
            found = walk(c)
            if found:
                return found
        return None
    return walk(d.screen().root)


def click(d, w):
    g, root = w.get_geometry(), d.screen().root
    p = root.translate_coords(w, g.width // 2, g.height // 2)
    xtest.fake_input(d, X.MotionNotify, x=p.x, y=p.y)
    d.sync(); time.sleep(0.05)
    xtest.fake_input(d, X.ButtonPress, 1)
    d.sync(); time.sleep(0.05)
    xtest.fake_input(d, X.ButtonRelease, 1)
    d.sync(); time.sleep(0.1)


def up(program):
    host = dict(os.environ, DISPLAY=":0")
    auth = sorted(f for f in os.listdir("/run/user/%d" % os.getuid()) if f.startswith(".mutter-Xwaylandauth."))
    if auth:
        host["XAUTHORITY"] = "/run/user/%d/%s" % (os.getuid(), auth[0])
    xe = subprocess.Popen(["Xephyr", DISP, "-screen", "440x540", "-nolisten", "tcp", "-title", "MORAI weather/time"],
                          env=host, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    for _ in range(50):
        try:
            d = display.Display(DISP)
            break
        except Exception:
            time.sleep(0.1)
    app = subprocess.Popen([program], env=dict(os.environ, DISPLAY=DISP), cwd=os.path.dirname(program),
                           stdout=open(os.path.expanduser("~/morai-eval-wtc.log"), "w"), stderr=subprocess.STDOUT,
                           start_new_session=True)
    open(PIDS, "w").write("%d %d\n" % (xe.pid, app.pid))
    for _ in range(100):                                 # the onefile bundle unpacks first (~2 s)
        f = frames(d)
        if f:
            break
        time.sleep(0.2)
    else:
        sys.exit("no program window")
    click(d, f[0][4])                                    # Connect
    print("connect pressed")


def main():
    cmd = sys.argv[1]
    if cmd == "up":
        up(os.path.abspath(sys.argv[2]))
    elif cmd == "down":
        if os.path.exists(PIDS):
            for pid in open(PIDS).read().split():
                try:
                    os.killpg(int(pid), 15)
                except OSError:
                    pass
            os.remove(PIDS)
    elif cmd == "set":
        d = display.Display(DISP)
        f = frames(d)
        click(d, f[2][WEATHERS[sys.argv[2].upper()]])
        click(d, f[1][HOURS[int(sys.argv[3])]])
        print("pressed", sys.argv[2], sys.argv[3])
    elif cmd == "tree":
        d = display.Display(DISP)

        def show(w, ind=""):
            for c in w.query_tree().children:
                g = c.get_geometry()
                print(ind, hex(c.id), g.x, g.y, g.width, g.height, c.get_attributes().map_state, c.get_wm_class())
                show(c, ind + "  ")
        show(d.screen().root)
    elif cmd == "shot":
        from PIL import Image
        d = display.Display(DISP)
        r = d.screen().root
        g = r.get_geometry()
        raw = r.get_image(0, 0, g.width, g.height, X.ZPixmap, 0xffffffff)
        Image.frombytes("RGB", (g.width, g.height), raw.data, "raw", "BGRX").save(sys.argv[2])


if __name__ == "__main__":
    main()
