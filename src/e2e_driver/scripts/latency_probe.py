#!/usr/bin/env python3
"""Closed-loop wiring and latency probe (run beside the stack for a test drive; writes ~out JSON at shutdown).

  camera -> /control_cmd   e2e_driver debug latency_ms: command publish - receipt stamp of the newest front frame used
  /control_cmd -> filter   /control_cmd arrival -> the next /morai_udp_bridge/control_cmd arrival (speed_filter forward)
  filter -> UDP send       that arrival -> the next control TX send in /diagnostics/high_hz (bridge 50 Hz loop)
  MORAI applied            competition_status accel / brake / speed: the command reached the simulator and the car moved
"""
import json

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from interfaces.msg import CompetitionVehicleStatus
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import String


def stats(x):
    x = np.asarray(x, float)
    if not len(x):
        return None
    return dict(n=int(len(x)), median=round(float(np.median(x)), 2), p95=round(float(np.percentile(x, 95)), 2),
                max=round(float(x.max()), 2))


def main():
    rospy.init_node("latency_probe")
    out = rospy.get_param("~out", "/tmp/latency_probe.json")
    ev = dict(debug=[], cmd=[], fwd=[], tx=[], status=[])
    now = rospy.get_time
    rospy.Subscriber("/e2e_driver/debug", String, lambda m: ev["debug"].append(json.loads(m.data)), queue_size=50)
    rospy.Subscriber("/control_cmd", CtrlCmd, lambda m: ev["cmd"].append((now(), m.accel, m.brake, m.steering)), queue_size=200, tcp_nodelay=True)
    rospy.Subscriber("/morai_udp_bridge/control_cmd", CtrlCmd, lambda m: ev["fwd"].append((now(), m.accel, m.brake, m.steering)),
                     queue_size=200, tcp_nodelay=True)

    def on_diag(m):                       # one high_hz status per control send: name morai_udp_bridge/control_tx, stamp = send time
        for s in m.status:
            if s.name == "morai_udp_bridge/control_tx":
                ev["tx"].append((m.header.stamp.to_sec(), s.message, {kv.key: kv.value for kv in s.values if kv.key in ("sent", "command_age_ms")}))
    rospy.Subscriber("/diagnostics/high_hz", DiagnosticArray, on_diag, queue_size=500, tcp_nodelay=True)
    rospy.Subscriber("/vehicle/competition_status", CompetitionVehicleStatus,
                     lambda m: ev["status"].append((m.header.stamp.to_sec(), m.signed_velocity_mps, m.accel, m.brake, m.steering_angle_rad, m.control_mode)),
                     queue_size=50)
    rospy.spin()

    d = ev["debug"]
    cmd, fwd = np.array(ev["cmd"]).reshape(-1, 4), np.array(ev["fwd"]).reshape(-1, 4)
    tx, last = [], None                   # keep the diagnostics whose sent counter moved (one per UDP send)
    for t, _, kv in ev["tx"]:
        if kv.get("sent") != last: tx.append(t); last = kv.get("sent")
    tx = np.array(tx)
    filt = []
    for t in cmd[:, 0]:
        j = np.searchsorted(fwd[:, 0], t)
        if j < len(fwd) and fwd[j, 0] - t < 0.05: filt.append(1e3 * (fwd[j, 0] - t))
    send = []
    for t in fwd[:, 0]:
        j = np.searchsorted(tx, t)
        if j < len(tx) and tx[j] - t < 0.05: send.append(1e3 * (tx[j] - t))
    st = np.array(ev["status"]).reshape(-1, 6)
    res = dict(
        steps=len(d),
        camera_to_control_cmd_ms=stats([x["latency_ms"] for x in d]),
        assemble_ms=stats([x["assemble_ms"] for x in d]), server_ms=stats([x["server_ms"] for x in d]),
        server_preprocess_ms=stats([x["server"].get("preprocess_ms") for x in d if x["server"].get("preprocess_ms") is not None]),
        server_model_ms=stats([x["server"].get("model_ms") for x in d if x["server"].get("model_ms") is not None]),
        newest_camera_age_at_t0_ms=stats([max(x["camera_age_ms"]) for x in d]),
        control_cmd_to_filter_out_ms=stats(filt), filter_out_to_udp_send_ms=stats(send),
        control_cmd_hz=round(len(cmd) / max(cmd[-1, 0] - cmd[0, 0], 1e-9), 1) if len(cmd) > 1 else None,
        filter_out_hz=round(len(fwd) / max(fwd[-1, 0] - fwd[0, 0], 1e-9), 1) if len(fwd) > 1 else None,
        speed_mps=dict(max=round(float(st[:, 1].max()), 2), last=round(float(st[-1, 1]), 2)) if len(st) else None,
        applied_accel_max=round(float(st[:, 2].max()), 3) if len(st) else None,
        control_mode=sorted({int(x) for x in st[:, 5]}) if len(st) else None,
        problems=sorted({p for x in d for p in x["problems"]})[:20],
        tx_messages=sorted({m for _, m, _ in ev["tx"]})[:10])
    json.dump(res, open(out, "w"), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
