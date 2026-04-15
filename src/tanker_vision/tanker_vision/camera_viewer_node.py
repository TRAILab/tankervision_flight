#!/usr/bin/env python3
import os
import threading
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image

_DISPLAY_HEIGHT = 540  # both panels are scaled to this height for display


class CameraViewerNode(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')

        self.declare_parameter('show_lucid',  True)
        self.declare_parameter('show_analog', True)
        self.declare_parameter('save_dir', os.path.expanduser('~/saved_frames'))

        self._show_lucid  = self.get_parameter('show_lucid').value
        self._show_analog = self.get_parameter('show_analog').value
        self._save_dir    = self.get_parameter('save_dir').value

        if not self._show_lucid and not self._show_analog:
            raise RuntimeError("At least one of 'show_lucid' or 'show_analog' must be True")

        self._bridge = CvBridge()
        self._lock   = threading.Lock()
        self._frame_lucid  = None
        self._frame_analog = None

        os.makedirs(self._save_dir, exist_ok=True)

        if self._show_lucid:
            self.create_subscription(Image, '/cam0/image_raw', self._lucid_cb,  10)
            self.get_logger().info('Subscribed to /cam0/image_raw  (Lucid)')

        if self._show_analog:
            self.create_subscription(Image, '/cam1/image_raw', self._analog_cb, 10)
            self.get_logger().info('Subscribed to /cam1/image_raw  (Analog)')

        self.get_logger().info("Press 's' to save frames, 'q' or ESC to quit")

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _lucid_cb(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'Lucid bridge error: {e}')
            return
        with self._lock:
            self._frame_lucid = frame

    def _analog_cb(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'Analog bridge error: {e}')
            return
        with self._lock:
            self._frame_analog = frame

    # ── Display ───────────────────────────────────────────────────────────────

    def get_frames(self):
        with self._lock:
            lucid  = self._frame_lucid.copy()  if self._frame_lucid  is not None else None
            analog = self._frame_analog.copy() if self._frame_analog is not None else None
        return lucid, analog

    def build_display(self, lucid, analog):
        panels = []
        if self._show_lucid:
            frame = lucid if lucid is not None else _placeholder('Lucid')
            panels.append(_resize_to_height(_label(frame, 'Lucid'), _DISPLAY_HEIGHT))
        if self._show_analog:
            frame = analog if analog is not None else _placeholder('Analog')
            panels.append(_resize_to_height(_label(frame, 'Analog'), _DISPLAY_HEIGHT))

        return panels[0] if len(panels) == 1 else np.hstack(panels)

    # ── Save ──────────────────────────────────────────────────────────────────

    def save_frames(self, lucid, analog):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        saved = []

        if self._show_lucid:
            if lucid is not None:
                path = os.path.join(self._save_dir, f'lucid_{ts}.png')
                cv2.imwrite(path, lucid)
                saved.append(path)
            else:
                self.get_logger().warn('No Lucid frame to save yet')

        if self._show_analog:
            if analog is not None:
                path = os.path.join(self._save_dir, f'analog_{ts}.png')
                cv2.imwrite(path, analog)
                saved.append(path)
            else:
                self.get_logger().warn('No Analog frame to save yet')

        for p in saved:
            self.get_logger().info(f'Saved: {p}')


# ── Module-level helpers ──────────────────────────────────────────────────────

def _placeholder(label: str) -> np.ndarray:
    img = np.zeros((_DISPLAY_HEIGHT, 960, 3), dtype=np.uint8)
    cv2.putText(img, f'Waiting for {label}...', (20, _DISPLAY_HEIGHT // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 180), 2)
    return img


def _resize_to_height(img: np.ndarray, height: int) -> np.ndarray:
    h, w = img.shape[:2]
    return cv2.resize(img, (int(w * height / h), height))


def _label(img: np.ndarray, text: str) -> np.ndarray:
    out = img.copy()
    cv2.putText(out, text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    return out


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    cv2.namedWindow('Camera Viewer', cv2.WINDOW_NORMAL)

    try:
        while rclpy.ok():
            lucid, analog = node.get_frames()
            cv2.imshow('Camera Viewer', node.build_display(lucid, analog))

            key = cv2.waitKey(30) & 0xFF
            if key == ord('s'):
                node.save_frames(lucid, analog)
            elif key in (ord('q'), 27):  # q or ESC
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
