#!/usr/bin/env python3
"""Closed-loop test probe on a live MORAI (measurement only; GT comes from the judge-only Ego status, never fed to the
stack): rates, lidar_leveler pass ratio, localization output gaps and error, route progress / nav text vs GT, RTF.
GT pose: /collect_only/ego_status (collect-only-ego-status-node.py on UDP 9095). GT route s: RouteTracker on that pose
(the scorer's KMap.progress, which made the training path_s). RTF: sum |d GT position| / sum(v dt) over wall time while
moving (> 1 m/s). Writes ~out JSON at shutdown (Ctrl-C)."""
import json
import os
import sys

import numpy as np
import rospy
from interfaces.msg import CompetitionVehicleStatus, LidarBevScan, SectionAnnotation
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, NavSatFix, PointCloud2
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from e2e_driver.route import Odometer, Route, RouteTracker  # noqa: E402


def yaw_of(q):
    return float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def st(x):
    x = np.asarray(x, float)
    return None if not len(x) else dict(n=int(len(x)), median=round(float(np.median(x)), 3), p95=round(float(np.percentile(x, 95)), 3),
                                        max=round(float(x.max()), 3))


def main():
    rospy.init_node("live_probe")
    out, route = rospy.get_param("~out"), Route(rospy.get_param("~route"))
    now = rospy.get_time
    ev = {k: [] for k in ("loc", "gt", "st", "gps", "sec", "cloud", "lev", "bev", "front", "dbg")}
    pose = lambda m: (m.header.stamp.to_sec(), m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
    rospy.Subscriber("/localization/ego", Odometry, lambda m: ev["loc"].append(pose(m)), queue_size=200)
    rospy.Subscriber("/collect_only/ego_status", Odometry, lambda m: ev["gt"].append(pose(m)), queue_size=200)
    rospy.Subscriber("/vehicle/competition_status", CompetitionVehicleStatus,
                     lambda m: ev["st"].append((m.header.stamp.to_sec(), m.signed_velocity_mps, m.accel, m.brake)), queue_size=200)
    rospy.Subscriber("/sensors/gps/fix", NavSatFix, lambda m: ev["gps"].append(
        (m.header.stamp.to_sec(), int(m.status.status < 0 or (m.latitude == 0 and m.longitude == 0)))), queue_size=200)
    rospy.Subscriber("/section_annotation", SectionAnnotation, lambda m: ev["sec"].append((now(), m.state)), queue_size=200)
    rospy.Subscriber("/velodyne_points", PointCloud2, lambda m: ev["cloud"].append(now()), queue_size=20)
    rospy.Subscriber("/velodyne_points/leveled", PointCloud2, lambda m: ev["lev"].append(now()), queue_size=20)
    rospy.Subscriber("/lidar_bev/scan", LidarBevScan, lambda m: ev["bev"].append(now()), queue_size=20)
    rospy.Subscriber("/image/front/compressed", CompressedImage, lambda m: ev["front"].append(now()), queue_size=5)
    rospy.Subscriber("/e2e_driver/debug", String, lambda m: ev["dbg"].append(json.loads(m.data)), queue_size=50)
    t_start = now()
    rospy.spin()
    t_end = now()
    span = max(t_end - t_start, 1e-9)
    loc, gt, sv, gps = (np.array(ev[k]).reshape(-1, n) for k, n in (("loc", 4), ("gt", 4), ("st", 4), ("gps", 2)))
    sec = np.array(ev["sec"]).reshape(-1, 2)
    sec_at = lambda t: int(sec[np.clip(np.searchsorted(sec[:, 0], t) - 1, 0, len(sec) - 1), 1]) if len(sec) else 0
    res = dict(seconds=round(span, 1), rates_hz={k: round(len(ev[k]) / span, 2) for k in ("loc", "gt", "st", "gps", "cloud", "lev", "bev", "front")},
               leveler_pass=round(len(ev["lev"]) / max(len(ev["cloud"]), 1), 3), sections_seen=sorted({int(s) for s in sec[:, 1]}) if len(sec) else [])
    if len(gps) > 1:
        res["gps_blackout_s"] = round(float(np.sum(np.diff(gps[:, 0]) * gps[:-1, 1])), 1)
    if len(loc) > 1:
        e = np.diff(np.r_[t_start, loc[:, 0], t_end]); res["loc_gaps"] = dict(max_s=round(float(e.max()), 2), over_0p2s_total=round(float(e[e > 0.2].sum()), 1))
        in15 = np.array([sec_at(t) == 15 for t in loc[:, 0]])
        if in15.any():
            t15 = loc[in15, 0]; e15 = np.diff(t15); res["loc_gaps_section15"] = dict(messages=int(in15.sum()), max_s=round(float(e15.max()), 2) if len(e15) else None)
    if len(loc) and len(gt) > 1:
        err = np.hypot(loc[:, 1] - np.interp(loc[:, 0], gt[:, 0], gt[:, 1]), loc[:, 2] - np.interp(loc[:, 0], gt[:, 0], gt[:, 2]))
        in15 = np.array([sec_at(t) == 15 for t in loc[:, 0]])
        res["loc_err_m"] = dict(all=st(err), section15=st(err[in15]) if in15.any() else None)
    if len(gt) > 1 and len(sv) > 1:
        tr, odo = RouteTracker(route), Odometer()
        s_gt = np.array([tr.update(t, x, y, h, 0.0) for t, x, y, h in gt])
        d = [x for x in ev["dbg"] if x.get("s") is not None]
        if d:
            tq = np.array([x["t0"] for x in d]); sn = np.array([x["s"] for x in d]); sg = np.interp(tq, gt[:, 0], s_gt)
            err = (sn - sg + route.lap / 2) % route.lap - route.lap / 2
            nav = np.array([x["nav"] == route.nav_text(float(s)) for x, s in zip(d, sg)])
            res["route_s_err_m"] = st(np.abs(err)); res["nav_equal"] = round(float(nav.mean()), 4)
        step = np.hypot(np.diff(gt[:, 1]), np.diff(gt[:, 2]))
        v = np.interp(gt[1:, 0], sv[:, 0], sv[:, 1]); dt = np.diff(gt[:, 0]); mv = v > 1.0
        if mv.sum() > 10:
            res["rtf"] = round(float(step[mv].sum() / np.sum(v[mv] * dt[mv])), 3)
        res["speed_max_mps"] = round(float(sv[:, 1].max()), 2); res["applied_accel_max"] = round(float(sv[:, 2].max()), 3)
        res["distance_m"] = round(float(step.sum()), 1)
    json.dump(res, open(out, "w"), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
