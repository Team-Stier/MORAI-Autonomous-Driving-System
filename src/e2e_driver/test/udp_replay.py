"""Send a bag's /velodyne_packets to UDP 127.0.0.1:2368 at their recorded pace (bag-in-the-loop wiring test of vlp16_bev).
Usage: python3 udp_replay.py [--start S] [--duration D] BAG [BAG ...]   (start/duration in seconds from the first bag start,
matching rosbag play -s/-u)"""
import argparse, socket, time

import rosbag


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--start", type=float, default=0); ap.add_argument("--duration", type=float, default=1e9)
    ap.add_argument("--port", type=int, default=2368); ap.add_argument("bags", nargs="+")
    a = ap.parse_args()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t_bag0 = rosbag.Bag(a.bags[0]).get_start_time() + a.start
    wall0, n = time.time(), 0
    for p in a.bags:
        for _, m, t in rosbag.Bag(p).read_messages(topics=["/velodyne_packets"]):
            for q in m.packets:
                dt = q.stamp.to_sec() - t_bag0
                if dt < 0: continue
                if dt > a.duration: print("sent", n); return
                wait = wall0 + dt - time.time()
                if wait > 0: time.sleep(wait)
                s.sendto(bytes(q.data), ("127.0.0.1", a.port)); n += 1
    print("sent", n)


if __name__ == "__main__":
    main()
