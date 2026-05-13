#!/usr/bin/env python3
"""
maxvis_record_node — Records analog camera (MaxVis) frames and IMU to rosbag.

Records continuously while the analog signal is present (frame is not blue).
Stops recording when the MaxVis is off (frame is solid blue — no signal from DFG).
Starts a new bag each time signal returns.

Bags are written to: <session_path>/maxvis_YYYYMMDD_HHMMSS/

Config keys (under maxvis_recording):
    blue_dominance_ratio: float  — B/(R+G) ratio above which frame is considered blue (default 1.5)
    blue_mean_threshold:  int    — minimum mean blue value to trigger check (default 80)
"""
import os
import subprocess
import threading
import time
import yaml
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


_LATCHED_QOS = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _deep_merge(base: dict, override: dict) -> dict:
    result = dict(base)
    for k, v in override.items():
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = _deep_merge(result[k], v)
        else:
            result[k] = v
    return result


def _load_config(node) -> dict:
    cfg = {}
    for param in ('config', 'unit_config'):
        try:
            path = node.get_parameter(param).get_parameter_value().string_value
        except Exception:
            continue
        if path and os.path.exists(path):
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            cfg = _deep_merge(cfg, data)
    return cfg


class MaxvisRecordNode(Node):
    def __init__(self):
        super().__init__('maxvis_record_node')
        self.declare_parameter('config', '')
        self.declare_parameter('unit_config', '')

        self._cfg = _load_config(self)
        _mv_cfg = self._cfg.get('maxvis_recording', {})
        self._blue_ratio     = float(_mv_cfg.get('blue_dominance_ratio', 1.5))
        self._blue_threshold = int(_mv_cfg.get('blue_mean_threshold', 80))

        self._bridge        = CvBridge()
        self._session_path  = None
        self._recording     = False
        self._bag_proc      = None
        self._lock          = threading.Lock()

        self._status_pub = self.create_publisher(String, '/maxvis_record/status',10)
        self.create_timer(1.0, self._publish_status_tick)

        # Session path — latched, wait for status_node to publish it
        self.create_subscription(
            String,
            '/tankervision/session_path',
            self._session_path_cb,
            _LATCHED_QOS,
        )

        # Analog camera frames
        self.create_subscription(
            Image,
            '/cam1/image_raw',
            self._image_cb,
            _BEST_EFFORT_QOS,
        )

        self.get_logger().info(
            f'maxvis_record_node started — '
            f'blue_ratio={self._blue_ratio}, blue_threshold={self._blue_threshold}'
        )


    def _session_path_cb(self, msg: String):
        self._session_path = msg.data
        self.get_logger().info(f'Session path: {self._session_path}')

    def _is_blue_frame(self, img_msg: Image) -> bool:
        """Return True if the frame is the solid-blue no-signal frame from DFG."""
        try:
            frame = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            mean_b = float(np.mean(frame[:, :, 0]))
            mean_g = float(np.mean(frame[:, :, 1]))
            mean_r = float(np.mean(frame[:, :, 2]))
            if mean_b < self._blue_threshold:
                return False
            return mean_b > self._blue_ratio * mean_r and mean_b > self._blue_ratio * mean_g
        except Exception as e:
            self.get_logger().warn(f'Blue frame check failed: {e}')
            return False

    def _image_cb(self, msg: Image):
        if self._session_path is None:
            return

        is_blue = self._is_blue_frame(msg)

        with self._lock:
            if not is_blue and not self._recording:
                self._start_recording()
            elif is_blue and self._recording:
                self._stop_recording()
                
    def _publish_status_tick(self):
        msg = String()
        msg.data = 'RECORDING' if self._recording else 'SCANNING'
        self._status_pub.publish(msg)
        
    def _start_recording(self):
        """Start a new rosbag for this signal segment. Must be called with _lock held."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_path  = os.path.join(self._session_path, f'maxvis_{timestamp}')
        os.makedirs(self._session_path, exist_ok=True)

        cmd = [
            'ros2', 'bag', 'record',
            '-o', bag_path,
            '/cam1/image_throttled',
            '/im19/imu',
            '/cam0/image_raw',
            '--compression-mode', 'message',
            '--compression-format', 'zstd',
        ]

        env = os.environ.copy()
        self._bag_proc = subprocess.Popen(cmd, env=env)
        self._recording = True
        self.get_logger().info(f'MaxVis recording started: {bag_path}')

    def _stop_recording(self):
        """Stop the current rosbag. Must be called with _lock held."""
        if self._bag_proc is not None:
            self._bag_proc.terminate()
            threading.Thread(
                target=self._bag_proc.wait, daemon=True
            ).start()
            self._bag_proc = None
        self._recording = False
        self.get_logger().info('MaxVis recording stopped — signal lost')

    def destroy_node(self):
        with self._lock:
            if self._recording:
                self._stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MaxvisRecordNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()