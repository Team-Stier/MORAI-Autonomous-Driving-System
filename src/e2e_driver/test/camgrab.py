"""One frame per camera -> /data/<prefix>_<cam>.jpg, with format, bytes, decoded size and brightness (chroot).
  live:  python3 camgrab.py PREFIX                     (/image/<cam>/compressed from the bridge)
  bag:   python3 camgrab.py PREFIX BAG [TOPIC_SUFFIX]   (first frames; collection bags use _10hz)
Compare encodings with the JPEG quantization tables (same DQT = same quality setting)."""
import sys

import cv2
import numpy as np

CAMS = ("front", "left", "right", "camera_7")


def report(pre, cam, m):
    img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
    open("/data/%s_%s.jpg" % (pre, cam), "wb").write(m.data)
    print(cam, m.format, len(m.data), "bytes", img.shape, "mean %.1f std %.1f" % (img.mean(), img.std()))


def main():
    pre = sys.argv[1]
    if len(sys.argv) > 2:
        import rosbag
        want = {"/image/%s/compressed%s" % (c, sys.argv[3] if len(sys.argv) > 3 else "_10hz"): c for c in CAMS}
        for topic, m, _ in rosbag.Bag(sys.argv[2]).read_messages(topics=list(want)):
            if topic in want:
                report(pre, want.pop(topic), m)
            if not want:
                break
    else:
        import rospy
        from sensor_msgs.msg import CompressedImage
        rospy.init_node("camgrab", anonymous=True)
        for c in CAMS:
            report(pre, c, rospy.wait_for_message("/image/%s/compressed" % c, CompressedImage, timeout=20))


if __name__ == "__main__":
    main()
