#!/usr/bin/env python3
"""GUI regression check after building, using a private ROS master and RViz window."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client


def wait_for(condition, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = condition()
        if result:
            return result
        time.sleep(0.05)
    raise AssertionError("traffic signal overlay check timed out")


def main():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:" + str(port)
    os.environ["DISABLE_ROS1_EOL_WARNINGS"] = "1"
    root = Path(__file__).resolve().parents[4]
    processes = []
    with tempfile.TemporaryFile(mode="w+") as log:
        try:
            processes.append(subprocess.Popen(["roscore", "-p", str(port)], stdout=log,
                                              stderr=log, start_new_session=True))

            def master_ready():
                try:
                    return xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"]).getUri("check")[0] == 1
                except OSError:
                    return False

            wait_for(master_ready)
            import rospy
            from PIL import Image, ImageChops
            from PyQt5.QtGui import QImage
            from PyQt5.QtWidgets import QApplication
            from visualization_msgs.msg import Marker

            application = QApplication([])
            rospy.init_node("check_traffic_signal_overlay", anonymous=True, disable_signals=True)
            publisher = rospy.Publisher("/visualizer/traffic_light", Marker, queue_size=1)
            rviz = subprocess.Popen(["rviz", "-d", str(root / "src/visualizer/rviz/visualizer.rviz"),
                                     "__name:=traffic_signal_overlay_check"], stdout=log,
                                    stderr=log, start_new_session=True)
            processes.append(rviz)
            wait_for(lambda: publisher.get_num_connections() > 0)
            marker = Marker()
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.text = "Traffic light: GREEN"
            marker.color.g = marker.color.a = 1
            marker.lifetime = rospy.Duration(1)
            heartbeat = rospy.Timer(rospy.Duration(0.1), lambda event: publisher.publish(marker))

            def find_window():
                result = subprocess.run(["xdotool", "search", "--onlyvisible", "--pid", str(rviz.pid),
                                         "--name", "RViz"], text=True, stdout=subprocess.PIPE)
                return result.stdout.splitlines()[0] if result.returncode == 0 else None

            window = wait_for(find_window)

            def capture():
                application.processEvents()
                pixels = application.primaryScreen().grabWindow(int(window)).toImage()
                pixels = pixels.convertToFormat(QImage.Format_RGB888)
                return Image.frombytes("RGB", (pixels.width(), pixels.height()),
                                       pixels.bits().asstring(pixels.byteCount()), "raw", "RGB",
                                       pixels.bytesPerLine())

            def text_bounds(image):
                red, green, blue = image.split()
                mask = ImageChops.multiply(green.point(lambda value: 255 if value > 180 else 0),
                                          red.point(lambda value: 255 if value < 80 else 0))
                mask = ImageChops.multiply(mask, blue.point(lambda value: 255 if value < 80 else 0))
                # RViz status icons elsewhere in the window are also green.
                mask.paste(0, (0, 0, image.width // 2, image.height))
                mask.paste(0, (0, image.height // 3, image.width, image.height))
                return mask.getbbox()

            gaps = []
            for width in (1000, 1300):
                subprocess.run(["xdotool", "windowsize", window, str(width), "760"], check=True)
                subprocess.run(["xdotool", "windowmove", window, "0", "0"], check=True)
                time.sleep(0.4)
                wait_for(lambda: text_bounds(capture()))
                image = capture()
                bounds = text_bounds(image)
                assert bounds[2] > image.width / 2 and bounds[1] < image.height / 3
                gaps.append((image.width - bounds[2], bounds[1]))
            assert abs(gaps[0][0] - gaps[1][0]) <= 3, "text did not stay anchored to the right"
            assert abs(gaps[0][1] - gaps[1][1]) <= 3, "text did not stay anchored to the top"
            subprocess.run(["xdotool", "mousemove", "--window", window, "650", "380", "click", "4"],
                           check=True)
            time.sleep(0.2)
            assert text_bounds(capture()) == bounds, "camera zoom moved the overlay"
            heartbeat.shutdown()
            time.sleep(1.3)
            assert text_bounds(capture()) is None, "missing input left stale text visible"
            assert rviz.poll() is None, "RViz stopped after missing input"
            print("PASS: traffic signal top-right anchoring on resize/zoom and missing-input expiry")
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
            log.seek(0)
            Path("/tmp/traffic-signal-overlay-check.log").write_text(log.read())


if __name__ == "__main__":
    main()
