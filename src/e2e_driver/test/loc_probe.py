#!/usr/bin/env python3
"""Localization / route-progress probe for bag_loop.sh (sim clock = bag time). Records the stack's /localization/ego, the
bag's GT pose (/collect_only/ego_status) and the collection stack's own EKF (/collect_only/localization_ekf, made with the
real velodyne), GPS fix status and e2e_driver's route s; at shutdown writes ~out JSON:
  output gaps of /localization/ego (overall and in GT section 15, the GPS-shade + tunnel section),
  position error vs GT (ours, and the collection EKF for reference), route s and nav text vs the training values (_ego
  path_s at the same bag time, nav text from the same route asset).
Params: ~out, ~ego (step-0 _ego npz of the episode), ~route (assets/route.json)."""
import json
import os
import sys

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from e2e_driver.route import Route  # noqa: E402


def st(x):
    x = np.asarray(x, float)
    return None if not len(x) else dict(n=int(len(x)), median=round(float(np.median(x)), 3), p95=round(float(np.percentile(x, 95)), 3),
                                        max=round(float(x.max()), 3))


def main():
    rospy.init_node("loc_probe")
    out, ego, route = rospy.get_param("~out"), rospy.get_param("~ego"), Route(rospy.get_param("~route"))
    ev = dict(loc=[], gt=[], ekf=[], gps=[], dbg=[])
    xy = lambda m: (m.header.stamp.to_sec(), m.pose.pose.position.x, m.pose.pose.position.y)
    rospy.Subscriber("/localization/ego", Odometry, lambda m: ev["loc"].append(xy(m)), queue_size=200)
    rospy.Subscriber("/collect_only/ego_status", Odometry, lambda m: ev["gt"].append(xy(m)), queue_size=200)
    rospy.Subscriber("/collect_only/localization_ekf", Odometry, lambda m: ev["ekf"].append(xy(m)), queue_size=200)
    rospy.Subscriber("/sensors/gps/fix", NavSatFix, lambda m: ev["gps"].append(
        (m.header.stamp.to_sec(), int(m.status.status < 0 or (m.latitude == 0 and m.longitude == 0)))), queue_size=200)
    rospy.Subscriber("/e2e_driver/debug", String, lambda m: ev["dbg"].append(json.loads(m.data)), queue_size=50)
    from interfaces.msg import SectionAnnotation
    ev["sec"], ev["cloud"], ev["leveled"] = [], [], []
    from sensor_msgs.msg import PointCloud2
    rospy.Subscriber("/velodyne_points", PointCloud2, lambda m: ev["cloud"].append((m.header.stamp.to_sec(), rospy.get_time(), m.width)), queue_size=20)
    rospy.Subscriber("/velodyne_points/leveled", PointCloud2, lambda m: ev["leveled"].append((m.header.stamp.to_sec(), rospy.get_time(), m.width)), queue_size=20)
    rospy.Subscriber("/section_annotation", SectionAnnotation, lambda m: ev["sec"].append((m.header.stamp.to_sec(), rospy.get_time(), m.state)), queue_size=200)
    rospy.spin()
    np.savez(out.replace(".json", ".npz"), **{k: np.array(v) for k, v in ev.items() if k != "dbg"},
             dbg=np.array([(x["t0"], x["s"] if x.get("s") is not None else np.nan) for x in ev["dbg"]]).reshape(-1, 2))

    z = dict(np.load(ego))                                  # NpzFile re-reads an array on every z[key]
    bt = z["bag_ns"].astype(np.float64) * 1e-9
    sec_at = lambda t: z["section"][np.clip(np.searchsorted(bt, t), 0, len(bt) - 1)]
    loc, gt, ekf = (np.array(ev[k]).reshape(-1, 3) for k in ("loc", "gt", "ekf"))
    gps = np.array(ev["gps"]).reshape(-1, 2)
    t0, t1 = (gt[0, 0], gt[-1, 0]) if len(gt) else (0, 0)
    res = dict(window_s=round(t1 - t0, 1), gps_blackout_s=round(float(np.sum(np.diff(gps[:, 0]) * gps[:-1, 1])), 1) if len(gps) > 1 else None)

    def gaps(t, lo, hi):
        t = t[(t >= lo) & (t <= hi)]
        e = np.diff(np.r_[lo, t, hi])
        return dict(messages=int(len(t)), max_gap_s=round(float(e.max()), 3), time_in_gaps_over_0p2s=round(float(e[e > 0.2].sum()), 2))
    in15 = np.flatnonzero(z["section"] == 15)
    w15 = (bt[in15[0]], bt[in15[-1]]) if len(in15) else None
    res["loc_output"] = gaps(loc[:, 0], t0, t1)
    if w15: res["loc_output_section15"] = dict(gaps(loc[:, 0], max(w15[0], t0), min(w15[1], t1)), window_s=round(min(w15[1], t1) - max(w15[0], t0), 1))
    for name, a in (("ours", loc), ("collection_ekf", ekf)):
        if not len(a) or not len(gt): continue
        e = np.hypot(a[:, 1] - np.interp(a[:, 0], gt[:, 0], gt[:, 1]), a[:, 2] - np.interp(a[:, 0], gt[:, 0], gt[:, 2]))
        m15 = np.array([sec_at(t) == 15 for t in a[:, 0]])
        res["pos_err_m_" + name] = dict(all=st(e), section15=st(e[m15]))
    if len(loc) and len(ekf):
        res["ours_vs_collection_ekf_m"] = st(np.hypot(loc[:, 1] - np.interp(loc[:, 0], ekf[:, 0], ekf[:, 1]),
                                                      loc[:, 2] - np.interp(loc[:, 0], ekf[:, 0], ekf[:, 2])))
    d = [x for x in ev["dbg"] if x.get("s") is not None]
    if d:
        tq = np.array([x["t0"] for x in d]); s_node = np.array([x["s"] for x in d])
        s_gt = np.interp(tq, bt, z["path_s"].astype(np.float64))
        err = s_node - s_gt; m15 = np.array([sec_at(t) == 15 for t in tq])
        nav_eq = np.array([x["nav"] == route.nav_text(s) for x, s in zip(d, s_gt)])
        res["route_s_err_m"] = dict(all=st(np.abs(err)), section15=st(np.abs(err[m15])), mean_signed=round(float(err.mean()), 3))
        res["nav_text_equal"] = dict(all=round(float(nav_eq.mean()), 4), section15=round(float(nav_eq[m15].mean()), 4) if m15.any() else None)
        res["nav_mismatch_examples"] = [[x["nav"], route.nav_text(s), round(float(s), 1)] for x, s, ok in zip(d, s_gt, nav_eq) if not ok][:6]
    json.dump(res, open(out, "w"), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
