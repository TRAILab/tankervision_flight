#!/usr/bin/env python3
"""
record_data_node.py — TankerVision fire detection and recording node.
Flight mode only.

Subscribes to /tankervision/session_path to get the session folder.

On fire detection:
  - Publishes /camera/record_mode = "record"
    so arena_camera_node saves full-res raw images
  - Starts rosbag in session_folder/fire_<ts>/
  - Records: /arena_camera_node/images, /cam1/image_raw, /im19/imu

When recording timeout expires:
  - Stops rosbag
  - Publishes /camera/record_mode = "standby"
    so arena_camera_node stops saving raw images
"""

import os
import signal
import subprocess
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import yaml


_IMG_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=3,
)

_LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _load_yaml(path: str) -> dict:
    with open(path) as f:
        loaded = yaml.safe_load(f)
    return loaded if loaded is not None else {}


class RecordDataNode(Node):

    def __init__(self):
        super().__init__("record_data_node")

        # ── Load config ───────────────────────────────────────────
        default_cfg = os.path.join(
            os.path.expanduser("~"),
            "tankervision_flight",
            "config",
            "tankervision.yaml",
        )

        yaml_path = self.declare_parameter("config", default_cfg).value

        try:
            cfg = _load_yaml(yaml_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            cfg = {}

        trigger_cfg = cfg.get("trigger", {})
        yolo_cfg = cfg.get("yolo", {})

        self._timeout_sec = float(trigger_cfg.get("timeout_sec", 20))
        self._cooldown_sec = float(trigger_cfg.get("cooldown_sec", 5))
        self._fire_class = int(yolo_cfg.get("fire_class", 2))
        self._confidence = float(yolo_cfg.get("confidence", 0.3))

        # Resolve model path.
        model_path = yolo_cfg.get(
            "model",
            "src/tanker_vision/tanker_vision/fire_detection.pt",
        )

        if not os.path.isabs(model_path):
            repo_root = os.path.join(os.path.expanduser("~"), "tankervision_flight")
            model_path = os.path.join(repo_root, model_path)

        # ── Session path — wait for status_node to publish it ─────
        self._session_dir = None

        # ── Load YOLO ─────────────────────────────────────────────
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self._model = YOLO(model_path, verbose=False)
        self._bridge = CvBridge()

        # ── Recording state ───────────────────────────────────────
        self._is_recording = False
        self._recording_end = 0.0
        self._cooldown_until = 0.0
        self._process = None

        # ── Publishers ────────────────────────────────────────────
        self._record_mode_pub = self.create_publisher(
            String,
            "/camera/record_mode",
            10,
        )

        self._status_pub = self.create_publisher(
            String,
            "/record_data/status",
            10,
        )

        # ── Subscriptions ─────────────────────────────────────────
        self.create_subscription(
            String,
            "/tankervision/session_path",
            self._session_cb,
            _LATCHED_QOS,
        )

        # This should be the converted/downscaled BGR8 image from arena_camera_node.
        self.create_subscription(
            Image,
            "/arena_camera_node/images",
            self._image_cb,
            _IMG_QOS,
        )

        # ── Timers ────────────────────────────────────────────────
        self.create_timer(1.0, self._check_recording_status)

        self.get_logger().info(
            f"RecordDataNode ready — timeout:{self._timeout_sec}s "
            f"cooldown:{self._cooldown_sec}s conf:{self._confidence}"
        )

    # ── Session path callback ─────────────────────────────────────
    def _session_cb(self, msg: String):
        if not self._session_dir:
            self._session_dir = msg.data
            self.get_logger().info(f"Session path: {self._session_dir}")

    # ── Image callback — YOLO inference ───────────────────────────
    def _image_cb(self, msg: Image):
        if self._session_dir is None:
            return

        now = time.time()

        if now < self._cooldown_until:
            self._publish_status("COOLDOWN")
            return

        try:
            cv_img = self._bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        results = self._model(
            cv_img,
            conf=self._confidence,
            verbose=False,
        )

        fire_detected = False

        if results and hasattr(results[0], "boxes"):
            for box in results[0].boxes:
                if int(box.cls) == self._fire_class:
                    fire_detected = True
                    break

        if fire_detected:
            self.get_logger().info("Fire detected")
            self._start_or_reset_recording()
            self._publish_status("RECORDING" if self._is_recording else "SCANNING")

    # ── Recording control ─────────────────────────────────────────
    def _start_or_reset_recording(self):
        now = time.time()

        if self._is_recording:
            self._recording_end = now + self._timeout_sec
            self.get_logger().info(
                f"Fire detected while recording — extended recording to "
                f"{self._timeout_sec}s from now"
            )
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_path = os.path.join(self._session_dir, f"fire_{ts}")

        os.makedirs(self._session_dir, exist_ok=True)

        cmd = [
            "/opt/ros/humble/bin/ros2",
            "bag",
            "record",
            "--storage",
            "mcap",
            "-o",
            bag_path,
            "/arena_camera_node/images",
            "/cam1/image_raw",
            "/im19/imu",
        ]

        self._process = subprocess.Popen(cmd)
        self._is_recording = True
        self._recording_end = now + self._timeout_sec

        self._publish_camera_record_mode("record")

        self.get_logger().info(f"Started rosbag: {bag_path}")

    def _check_recording_status(self):
        if self._is_recording and time.time() > self._recording_end:
            self._stop_recording()

    def _stop_recording(self):
        if self._process is not None:
            self.get_logger().info("Stopping rosbag...")

            self._process.send_signal(signal.SIGINT)

            try:
                self._process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(
                    "rosbag did not stop after SIGINT; terminating process"
                )
                self._process.terminate()

                try:
                    self._process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self.get_logger().warn(
                        "rosbag did not terminate; killing process"
                    )
                    self._process.kill()
                    self._process.wait()

            self._process = None

        self._publish_camera_record_mode("standby")

        self._is_recording = False
        self._cooldown_until = time.time() + self._cooldown_sec

        self.get_logger().info(
            f"Stopped rosbag — cooldown {self._cooldown_sec}s"
        )

        self._publish_status("SCANNING")

    def _publish_camera_record_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self._record_mode_pub.publish(msg)
        self.get_logger().info(f"Published /camera/record_mode: {mode}")

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def destroy_node(self):
        if self._process is not None:
            self._stop_recording()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = RecordDataNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()