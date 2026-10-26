#!/usr/bin/env python3
"""MJPEG view of the simulator window, for watching evaluation drives (runs on the simulator host; eval_manager.py
starts it on bc-250 with the system python3, which has python-xlib and Pillow).

    DISPLAY=:0 XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.* python3 viewer.py HOST PORT [FPS] [WIDTH]

http://HOST:PORT/ is a multipart JPEG stream of the largest window whose WM_CLASS contains "simulator", read through
XComposite (works on the headless Xwayland). Frames are captured only while someone is watching.
"""
import io
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from PIL import Image
from Xlib import X, display
from Xlib.ext import composite

HOST, PORT = sys.argv[1], int(sys.argv[2])
FPS = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
WIDTH = int(sys.argv[4]) if len(sys.argv) > 4 else 960
frame, cond, viewers = None, threading.Condition(), 0


def find(d):
    def walk(w):
        for c in w.query_tree().children:
            cls = c.get_wm_class()
            if cls and any("simulator" in s.lower() for s in cls) and c.get_attributes().map_state == X.IsViewable:
                yield c
            yield from walk(c)
    wins = sorted(walk(d.screen().root), key=lambda w: -(w.get_geometry().width * w.get_geometry().height))
    if not wins:
        raise LookupError("no simulator window")
    composite.redirect_window(wins[0], composite.RedirectAutomatic)
    return wins[0]


def capture():
    global frame
    d, w = display.Display(), None
    while True:
        if not viewers:
            time.sleep(0.2)
            continue
        t0 = time.time()
        try:
            w = w or find(d)
            g = w.get_geometry()
            pm = w.composite_name_window_pixmap()
            raw = pm.get_image(0, 0, g.width, g.height, X.ZPixmap, 0xffffffff)
            pm.free()
            img = Image.frombytes("RGB", (g.width, g.height), raw.data, "raw", "BGRX")
            buf = io.BytesIO()
            img.resize((WIDTH, round(g.height * WIDTH / g.width))).save(buf, "JPEG", quality=70)
            with cond:
                frame = buf.getvalue()
                cond.notify_all()
        except Exception as e:                       # the simulator is (re)starting: look for its window again
            print("capture:", e, flush=True)
            w = None
            time.sleep(1.0)
        time.sleep(max(0.0, 1.0 / FPS - (time.time() - t0)))


class Stream(BaseHTTPRequestHandler):
    def do_GET(self):
        global viewers
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        with cond:
            viewers += 1
        try:
            last = None
            while True:
                with cond:
                    cond.wait_for(lambda: frame is not last, timeout=5.0)
                    last = frame
                if last is not None:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n%s\r\n" % (len(last), last))
        except OSError:
            pass                                     # the viewer closed the page
        finally:
            with cond:
                viewers -= 1

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    threading.Thread(target=capture, daemon=True).start()
    ThreadingHTTPServer((HOST, PORT), Stream).serve_forever()
