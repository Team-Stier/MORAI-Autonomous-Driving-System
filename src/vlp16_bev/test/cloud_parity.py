"""Point cloud parity: collected VelodyneScan messages -> the real velodyne_pointcloud transform_node (reference, run only
for this test) and vlp16_bev.cloud.scan_cloud; compares every point. Needs a ROS master, transform_node running
(_model:=VLP16 _calibration:=<VLP16db.yaml> _min_range:=0.4 _max_range:=130) and velodyne_msgs importable.
Usage: python3 cloud_parity.py OUT_JSON N_SCANS BAG [BAG ...]"""
import json, sys, threading

import numpy as np
import rosbag, rospy
from sensor_msgs.msg import PointCloud2
from velodyne_msgs.msg import VelodyneScan

from vlp16_bev.cloud import DTYPE, FIELDS, scan_cloud


def main():
    out, n_max, bags = sys.argv[1], int(sys.argv[2]), sys.argv[3:]
    rospy.init_node("cloud_parity")
    got, ev = {}, threading.Event()

    def on_cloud(m):
        got[m.header.stamp.to_nsec()] = m; ev.set()
    rospy.Subscriber("/velodyne_points", PointCloud2, on_cloud, queue_size=10)
    pub = rospy.Publisher("/velodyne_packets", VelodyneScan, queue_size=10)
    rospy.sleep(2.0)
    res = dict(scans=0, missing=0, count_equal=0, fields_ok=None, ring_equal=0, max_abs={}, frac_bit_equal={})
    diffs = {k: [] for k in ("x", "y", "z", "intensity", "time")}; eq = {k: [] for k in diffs}
    for p in bags:
        for _, m, _ in rosbag.Bag(p).read_messages(topics=["/velodyne_packets"]):
            if res["scans"] >= n_max: break
            ev.clear(); pub.publish(m)
            key = m.header.stamp.to_nsec()
            for _ in range(40):
                if key in got: break
                ev.wait(0.05)
            res["scans"] += 1
            ref = got.pop(key, None)
            if ref is None: res["missing"] += 1; continue
            ns = np.array([q.stamp.to_nsec() for q in m.packets], np.int64)
            mine = scan_cloud(b"".join(bytes(q.data) for q in m.packets), ns, key)
            if res["fields_ok"] is None:
                res["fields_ok"] = ([(f.name, f.offset, f.datatype, f.count) for f in ref.fields] == [(n, o, t, 1) for n, o, t in FIELDS]
                                    and ref.point_step == DTYPE.itemsize and ref.height == 1 and ref.header.frame_id == "velodyne")
                res["ref_meta"] = dict(frame_id=ref.header.frame_id, point_step=ref.point_step, is_dense=bool(ref.is_dense),
                                       fields=[(f.name, f.offset, f.datatype) for f in ref.fields])
            r = np.frombuffer(bytes(ref.data), DTYPE, count=ref.width)
            if len(r) != len(mine): continue
            res["count_equal"] += 1
            res["ring_equal"] += int(np.array_equal(r["ring"], mine["ring"]))
            for k in diffs:
                diffs[k].append(float(np.abs(r[k].astype(np.float64) - mine[k]).max()) if len(r) else 0.0)
                eq[k].append(float((r[k] == mine[k]).mean()) if len(r) else 1.0)
            if res["scans"] % 100 == 0: print(res["scans"], flush=True)
    res["max_abs"] = {k: max(v) if v else None for k, v in diffs.items()}
    res["frac_bit_equal"] = {k: round(float(np.mean(v)), 8) if v else None for k, v in eq.items()}
    json.dump(res, open(out, "w"), indent=1); print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
