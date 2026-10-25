"""Send a bag's /velodyne_packets to UDP 127.0.0.1:2368 when the replay's /clock reaches each packet's recorded stamp
(bag-in-the-loop test of velodyne_driver -> vlp16_bev / velodyne_pointcloud), so the packets stay aligned with the
rosbag play --clock stream of the other sensors.
Usage: python3 udp_replay.py BAG [BAG ...]   (needs /use_sim_time and rosbag play --clock running)"""
import socket, sys, time

import rosbag, rospy


def main():
    rospy.init_node("udp_replay")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while not rospy.is_shutdown() and rospy.get_time() == 0:
        time.sleep(0.01)                                          # /clock not started yet
    n, late = 0, []
    for p in sys.argv[1:]:
        for _, m, _ in rosbag.Bag(p).read_messages(topics=["/velodyne_packets"]):
            for q in m.packets:
                ts = q.stamp.to_sec()
                now = rospy.get_time()
                if ts < now - 1.0: continue                       # before the played window (-s START)
                while not rospy.is_shutdown() and rospy.get_time() < ts:
                    time.sleep(0.001)
                if rospy.is_shutdown(): print("sent", n); return
                late.append(rospy.get_time() - ts)
                s.sendto(bytes(q.data), ("127.0.0.1", 2368)); n += 1
    late.sort()
    print("sent", n, "median lateness ms %.1f" % (1e3 * late[len(late) // 2] if late else -1))


if __name__ == "__main__":
    main()
