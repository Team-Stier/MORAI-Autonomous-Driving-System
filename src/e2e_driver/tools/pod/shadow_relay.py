#!/usr/bin/env python3
"""shadow_relay.py out PORT (in the closed-loop stack's master) | in PORT (in the shadow master, shadow.sh): the stack's
/vehicle/competition_status and /sensors/imu/data for the shadow collection driver, whose planners and control need
them. One rospy process talks to one master, so `out` subscribes (raw bytes, rospy.AnyMsg: the stack only gains a
subscriber) and sends each message as a UDP datagram to 127.0.0.1:PORT; `in` publishes them with header.stamp = its
wall-clock receipt time: the stack runs on simulator time (/use_sim_time), the shadow, like the collection stacks, on
wall time, and the planners drop inputs older than 0.5 s."""
import importlib
import socket
import sys

TOPICS = (("/vehicle/competition_status", "interfaces.msg", "CompetitionVehicleStatus"),
          ("/sensors/imu/data", "sensor_msgs.msg", "Imu"))


def out(port):
    import rospy
    rospy.init_node("shadow_relay_out", anonymous=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for i, (topic, _, _) in enumerate(TOPICS):
        rospy.Subscriber(topic, rospy.AnyMsg, lambda m, i=i: sock.sendto(bytes([i]) + m._buff, ("127.0.0.1", port)), queue_size=50)
    rospy.spin()


def inn(port):
    import rospy
    rospy.init_node("shadow_relay_in")
    classes = [getattr(importlib.import_module(mod), name) for _, mod, name in TOPICS]
    pubs = [rospy.Publisher(topic, c, queue_size=50) for (topic, _, _), c in zip(TOPICS, classes)]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", port))
    sock.settimeout(0.5)
    while not rospy.is_shutdown():
        try:
            data = sock.recv(65535)
        except socket.timeout:
            continue
        i = data[0]
        m = classes[i]().deserialize(data[1:])
        m.header.stamp = rospy.Time.now()
        pubs[i].publish(m)


if __name__ == "__main__":
    {"out": out, "in": inn}[sys.argv[1]](int(sys.argv[2]))
