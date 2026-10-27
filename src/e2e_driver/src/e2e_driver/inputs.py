"""Live sensor buffers -> one model request, picked the way morai_vla.data.MoraiDataset.sample picks a training row.

Per request time t0 (the clock of the stamps: ROS time live, the restored sim clock tau in the offline parity test):
  cameras  frame_targets(t0) = t0 - 0.3 ... t0 (nominal 0.1 s, no jitter at inference); per camera select_distinct over
           the buffered frames, each re-encoded as step 0 did (cv2 decode -> INTER_AREA resize -> JPEG q92); time text
           offsets = frame time - t0. Live, every frame is at or before t0; training rows also used frames up to 0.05 s
           after t0 (nearest to the target), so the newest offset here is always <= 0.
           Live (reuse=True), the history slots t0 - 0.1k take the frame an earlier step used as its current (t0) frame,
           the one closest to the target within REUSE_TOL, so its pixels (and the server's ViT output for them) repeat
           and each step brings one new frame per camera; a slot with no such frame (start, teleport, skipped steps)
           takes the nearest buffered frame as above.
  LiDAR    the same targets over the buffered scans (vlp16_bev per-scan BEV, stamped with the last packet): 4 scans,
           paired (k1 <- k0), (k3 <- k2) by the server's lidar_bev.pack_bev; times of k1 and k3.
  ego      competition_status signed speed and yaw (unwrapped) linearly interpolated at t0 - 1.5 ... t0 (16 points);
           the server integrates them with morai_vla.data.ego_trajectory. Training integrated the GT speed and yaw of the
           tau grid: competition_status speed equals the GT speed, and its yaw differs from the GT yaw by a constant
           0.0235 rad (run1024b-w1-001), which relative headings cancel.
  nav      route.RouteTracker s -> Route.nav_text.
"""
import math
import queue
import threading
from collections import deque

import cv2
import numpy as np

from e2e_driver.train_copy import DT, MAX_FRAME_GAP, MAX_SCAN_GAP, N_FRAMES, N_HIST, frame_targets, select_distinct

# s: an earlier current frame this close to a history target is reused. The targets are 0.1 s apart, so a reused frame is
# as far from its slot's target as it was from its own t0, where MAX_FRAME_GAP already applies (30 Hz cameras 40 ms late
# give 67 ms every step: a 50 ms window would re-pick, and re-encode, a newer frame for every slot)
REUSE_TOL = MAX_FRAME_GAP
CAMERAS = ("front", "left", "right", "camera_7")      # model order: Front, Front left, Front right, Front upward camera
SIZES = {"front": (448, 256), "left": (352, 256), "right": (352, 256), "camera_7": (256, 192)}


def reencode(data, size):
    """bridge JPEG -> the step-0 JPEG (tools/step0/step0_worker.py: same calls, same cv2 4.2 of the Noetic rootfs)."""
    im = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if im is None:
        return None
    ok, enc = cv2.imencode(".jpg", cv2.resize(im, size, interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, 92])
    return enc.tobytes()


class Inputs:
    def __init__(self, horizon=2.5, prefetch=False, reuse=True):
        """prefetch: re-encode every arriving camera frame in a worker thread per camera (cv2 releases the GIL), so a
        request only reads finished step-0 JPEGs; a frame the worker has not reached yet is encoded on the spot.
        reuse: history slots from the earlier steps' current frames (live); False: every slot as the training sample.
        on_encoded(camera, t, jpeg): called by the prefetch workers with each step-0 JPEG (the node streams it to the
        server ahead of the request)."""
        self.horizon, self.reuse, self.on_encoded = horizon, reuse, None
        self.used = {c: deque() for c in CAMERAS}       # (t, step-0 JPEG) of the frame each request used as current
        self.cams = {c: deque() for c in CAMERAS}        # (t, raw JPEG)
        self.cache = {}                                    # (camera, t) -> step-0 JPEG
        self.scans = deque()                               # (t, height, density, pose (3,), odom epoch)
        self.ego = deque()                                 # (t, v, unwrapped yaw)
        self._yaw = None                                   # (raw, unwrapped) of the last status
        self.lock, self.cache_lock = threading.Lock(), threading.Lock()
        self.work = {c: queue.Queue(8) for c in CAMERAS} if prefetch else None
        for c in CAMERAS if prefetch else ():
            threading.Thread(target=self._encode_loop, args=(c,), daemon=True).start()

    def _encode(self, cam, t, data):
        with self.cache_lock:
            if (cam, t) in self.cache:
                return self.cache[(cam, t)]
        jpeg = reencode(data, SIZES[cam])
        with self.cache_lock:
            self.cache[(cam, t)] = jpeg
        return jpeg

    def _encode_loop(self, cam):
        while True:
            t, data = self.work[cam].get()
            jpeg = self._encode(cam, t, data)
            if jpeg is not None and self.on_encoded is not None:
                self.on_encoded(cam, t, jpeg)

    def reset(self):
        """drop everything buffered (a teleport): requests wait until the horizon has filled again."""
        with self.lock:
            for q in list(self.cams.values()) + list(self.used.values()):
                q.clear()
            self.scans.clear(); self.ego.clear(); self._yaw = None
        with self.cache_lock:
            self.cache.clear()

    def _trim(self, q, t):
        while q and q[0][0] < t - self.horizon:
            q.popleft()

    def add_camera(self, cam, t, data):
        with self.lock:
            q = self.cams[cam]
            if q and t <= q[-1][0]:
                return
            q.append((t, bytes(data))); self._trim(q, t)
        if self.work is not None:
            try:
                self.work[cam].put_nowait((t, q[-1][1]))
            except queue.Full:
                pass                                          # behind: request() encodes what it picks

    def add_scan(self, t, height, density, pose, epoch=0):
        """-> True if kept (newer than the last scan)."""
        with self.lock:
            if self.scans and t <= self.scans[-1][0]:
                return False
            self.scans.append((t, height, density, np.asarray(pose, np.float64), epoch)); self._trim(self.scans, t)
            return True

    def add_status(self, t, v, yaw):
        with self.lock:
            if self.ego and t <= self.ego[-1][0]:
                return
            if self._yaw is None:
                u = yaw
            else:
                u = self._yaw[1] + math.atan2(math.sin(yaw - self._yaw[0]), math.cos(yaw - self._yaw[0]))
            self._yaw = (yaw, u)
            self.ego.append((t, v, u)); self._trim(self.ego, t)

    def latest(self, cam="front"):
        q = self.cams[cam]
        return q[-1][0] if q else None

    def request(self, t0, nav_text, lidar=True):
        """-> (request dict, problems list) or (None, problems) if an input is missing."""
        tgt = frame_targets(t0)
        problems = []
        with self.lock:
            cams = {c: list(q) for c, q in self.cams.items()}
            scans, ego = list(self.scans), list(self.ego)
        lo = min(q[0][0] for q in cams.values() if q) if any(cams.values()) else t0
        with self.cache_lock:
            self.cache = {k: v for k, v in self.cache.items() if k[1] >= lo}
        jpegs, times, stamps, newest, reused = [], [], [], [], 0
        for c in CAMERAS:
            if len(cams[c]) < N_FRAMES:
                return None, ["camera %s: %d frames" % (c, len(cams[c]))]
            tau = np.array([t for t, _ in cams[c]])
            k = np.maximum(select_distinct(tau, tgt), 0)
            tp, out = [float(tau[j]) for j in k], [None] * N_FRAMES
            if self.reuse:
                with self.lock:
                    used = list(self.used[c])
                newer = tp[-1]
                for i in range(N_FRAMES - 2, -1, -1):           # newest history slot first, each older than the next
                    cand = [(abs(t - tgt[i]), t, jpg) for t, jpg in used if t < newer and abs(t - tgt[i]) <= REUSE_TOL]
                    if cand:
                        _, tp[i], out[i] = min(cand, key=lambda x: x[:2]); reused += 1
                    elif tp[i] >= newer:                           # the raw pick is not older than a reused newer slot
                        older = np.nonzero(tau < newer)[0]
                        if not len(older):
                            return None, ["camera %s: no frame before %.3f" % (c, newer)]
                        k[i] = older[np.argmin(np.abs(tau[older] - tgt[i]))]; tp[i] = float(tau[k[i]])
                    newer = tp[i]
            tp = np.array(tp)
            if np.abs(tp - tgt).max() > MAX_FRAME_GAP:
                problems.append("camera %s: pick %.3f s from its target" % (c, float(np.abs(tp - tgt).max())))
            for i, j in enumerate(k):
                if out[i] is None:
                    out[i] = self._encode(c, float(tau[j]), cams[c][j][1])
                    if out[i] is None:
                        return None, ["camera %s: undecodable JPEG" % c]
            if self.reuse:
                with self.lock:
                    u = self.used[c]
                    if not u or u[-1][0] < tp[-1]:
                        u.append((float(tp[-1]), out[-1])); self._trim(u, t0)
            jpegs.append(out); times.append(tp - t0); stamps.append([float(x) for x in tp]); newest.append(float(tp[-1]))
        req = dict(t0=float(t0), camera_jpegs=jpegs, camera_times=np.stack(times), camera_stamps=stamps, camera_newest=newest, nav_text=nav_text,
                   pick=dict(reused=reused, err_ms=np.round(1e3 * (np.stack(times) - (tgt - t0)), 1).tolist()))
        if lidar:
            if len(scans) < N_FRAMES:
                return None, ["lidar: %d scans" % len(scans)]
            tau = np.array([s[0] for s in scans])
            k = np.maximum(select_distinct(tau, tgt), 0)
            if abs(tau[k[-1]] - t0) > MAX_SCAN_GAP:
                problems.append("lidar: newest pick %.3f s from t0" % float(tau[k[-1]] - t0))
            if len({scans[j][4] for j in k}) > 1:
                problems.append("lidar: odometry restarted inside the 4 scans")
            req.update(scan_height=np.stack([scans[j][1] for j in k]), scan_density=np.stack([scans[j][2] for j in k]),
                       scan_pose=np.stack([scans[j][3] for j in k]), bev_times=tau[k[[1, 3]]] - t0, scan_stamps=[float(tau[j]) for j in k])
        grid = t0 - DT * np.arange(N_HIST - 1, -1, -1)
        if len(ego) < 2 or ego[0][0] > grid[0] + DT:
            return None, ["ego: %.2f s of competition_status" % ((ego[-1][0] - ego[0][0]) if ego else 0.0)]
        e = np.array(ego)
        if t0 - e[-1, 0] > 0.2:
            problems.append("ego: newest competition_status %.3f s old" % float(t0 - e[-1, 0]))
        req.update(ego_v=np.interp(grid, e[:, 0], e[:, 1]), ego_yaw=np.interp(grid, e[:, 0], e[:, 2]))
        return req, problems
