#!/usr/bin/env python3
"""
camera_viewer_node.py

Displays Lucid and analog camera feeds in separate resizable windows.
Each window scales its feed to fill the window as you drag it larger.
A "Save" button is drawn in the corner of each window — click it or
press 's' to save both frames to ~/saved_frames/.

Topics:
  Lucid  → /arena_camera_node/images
  Analog → /v4l2_camera/image_raw

Controls:
  s / click Save button  - save current frames
  q / ESC                - quit
"""

import os
os.environ.setdefault('QT_QPA_FONTDIR', '/usr/share/fonts')  # suppress Qt font warning

import threading
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

# ── Topics ────────────────────────────────────────────────────────────────────
LUCID_TOPIC  = '/cam0/image_raw'
ANALOG_TOPIC = '/cam1/image_raw'

# ── QoS — match publisher (SensorDataQoS = best-effort, keep-last 5) ─────────
_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# ── Save button geometry (pixels from top-left of displayed frame) ─────────────
BTN_X, BTN_Y, BTN_W, BTN_H = 10, 10, 120, 40

# ── Encoding table ────────────────────────────────────────────────────────────
_ENCODING_MAP = {
    'bgr8':   (np.uint8,  3, None),
    'rgb8':   (np.uint8,  3, cv2.COLOR_RGB2BGR),
    'rgba8':  (np.uint8,  4, cv2.COLOR_RGBA2BGR),
    'bgra8':  (np.uint8,  4, cv2.COLOR_BGRA2BGR),
    'mono8':  (np.uint8,  1, cv2.COLOR_GRAY2BGR),
    'mono16': (np.uint16, 1, None),
    'yuv422': (np.uint8,  2, cv2.COLOR_YUV2BGR_YUYV),
    '16uc1':  (np.uint16, 1, None),
    '8uc1':   (np.uint8,  1, cv2.COLOR_GRAY2BGR),
    '8uc3':   (np.uint8,  3, None),
}


def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    """Decode a sensor_msgs/Image to BGR numpy array without cv_bridge."""
    encoding = msg.encoding.lower()
    if encoding not in _ENCODING_MAP:
        raise ValueError(f'Unsupported encoding: {msg.encoding}')

    dtype, channels, cvt = _ENCODING_MAP[encoding]
    arr = np.frombuffer(msg.data, dtype=dtype)
    arr = arr.reshape((msg.height, msg.width) if channels == 1
                      else (msg.height, msg.width, channels))

    if encoding in ('mono16', '16uc1'):
        arr = (arr >> 8).astype(np.uint8)
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

    if cvt is not None:
        arr = cv2.cvtColor(arr, cvt)

    return arr


def _fit_to_window(frame: np.ndarray, win_name: str) -> np.ndarray:
    """Scale frame to fill the current window size, preserving aspect ratio with black bars."""
    try:
        _, _, win_w, win_h = cv2.getWindowImageRect(win_name)
    except Exception:
        return frame

    if win_w <= 0 or win_h <= 0:
        return frame

    fh, fw = frame.shape[:2]
    scale  = min(win_w / fw, win_h / fh)
    new_w  = int(fw * scale)
    new_h  = int(fh * scale)

    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas  = np.zeros((win_h, win_w, 3), dtype=np.uint8)
    x_off   = (win_w - new_w) // 2
    y_off   = (win_h - new_h) // 2
    canvas[y_off:y_off + new_h, x_off:x_off + new_w] = resized
    return canvas


def _draw_save_button(img: np.ndarray, pressed: bool = False) -> np.ndarray:
    out   = img.copy()
    color = (0, 180, 0) if not pressed else (0, 255, 80)
    cv2.rectangle(out, (BTN_X, BTN_Y), (BTN_X + BTN_W, BTN_Y + BTN_H), color, -1)
    cv2.rectangle(out, (BTN_X, BTN_Y), (BTN_X + BTN_W, BTN_Y + BTN_H), (255, 255, 255), 2)
    cv2.putText(out, '[ Save ]', (BTN_X + 8, BTN_Y + 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return out


def _placeholder(label: str) -> np.ndarray:
    h, w = 480, 640
    img  = np.zeros((h, w, 3), dtype=np.uint8)
    cv2.putText(img, f'Waiting for {label}...', (20, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 180), 2)
    return img


def _label(img: np.ndarray, text: str) -> np.ndarray:
    out = img.copy()
    cv2.putText(out, text, (10, img.shape[0] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return out


def _in_button(x: int, y: int) -> bool:
    return BTN_X <= x <= BTN_X + BTN_W and BTN_Y <= y <= BTN_Y + BTN_H


# ── Mouse handler ─────────────────────────────────────────────────────────────

class _MouseState:
    def __init__(self):
        self.clicked = False

    def callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and _in_button(x, y):
            self.clicked = True


# ── ROS Node ──────────────────────────────────────────────────────────────────

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

        self._lock         = threading.Lock()
        self._frame_lucid  = None
        self._frame_analog = None

        os.makedirs(self._save_dir, exist_ok=True)

        if self._show_lucid:
            self.create_subscription(Image, LUCID_TOPIC, self._lucid_cb, _BEST_EFFORT_QOS)
            self.get_logger().info(f'Subscribed to {LUCID_TOPIC}  (Lucid)')

        if self._show_analog:
            self.create_subscription(Image, ANALOG_TOPIC, self._analog_cb, _BEST_EFFORT_QOS)
            self.get_logger().info(f'Subscribed to {ANALOG_TOPIC}  (Analog)')

        self.get_logger().info("Press 's' or click [ Save ] to save frames. 'q'/ESC to quit.")

    def _lucid_cb(self, msg: Image):
        try:
            frame = imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f'Lucid decode error: {e}')
            return
        with self._lock:
            self._frame_lucid = frame

    def _analog_cb(self, msg: Image):
        try:
            frame = imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f'Analog decode error: {e}')
            return
        with self._lock:
            self._frame_analog = frame

    def get_frames(self):
        with self._lock:
            lucid  = self._frame_lucid.copy()  if self._frame_lucid  is not None else None
            analog = self._frame_analog.copy() if self._frame_analog is not None else None
        return lucid, analog

    def save_frames(self, lucid, analog):
        ts    = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
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


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    mouse_states = {}

    if node._show_lucid:
        cv2.namedWindow('Lucid Camera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Lucid Camera', 960, 540)
        ms_lucid = _MouseState()
        cv2.setMouseCallback('Lucid Camera', ms_lucid.callback)
        mouse_states['lucid'] = ms_lucid

    if node._show_analog:
        cv2.namedWindow('Analog Camera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Analog Camera', 720, 480)
        ms_analog = _MouseState()
        cv2.setMouseCallback('Analog Camera', ms_analog.callback)
        mouse_states['analog'] = ms_analog

    save_flash = 0  # countdown: show button as "pressed" for N frames

    try:
        while rclpy.ok():
            lucid, analog = node.get_frames()

            # Check for save trigger (button click)
            do_save = False
            for ms in mouse_states.values():
                if ms.clicked:
                    ms.clicked = False
                    do_save = True

            if do_save:
                node.save_frames(lucid, analog)
                save_flash = 10

            pressed = save_flash > 0
            if save_flash > 0:
                save_flash -= 1

            # Lucid window
            if node._show_lucid:
                frame = _label(lucid if lucid is not None else _placeholder('Lucid'), 'Lucid')
                frame = _fit_to_window(frame, 'Lucid Camera')
                frame = _draw_save_button(frame, pressed)
                cv2.imshow('Lucid Camera', frame)

            # Analog window
            if node._show_analog:
                frame = _label(analog if analog is not None else _placeholder('Analog'), 'Analog')
                frame = _fit_to_window(frame, 'Analog Camera')
                frame = _draw_save_button(frame, pressed)
                cv2.imshow('Analog Camera', frame)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('s'):
                node.save_frames(lucid, analog)
                save_flash = 10
            elif key in (ord('q'), 27):
                break

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
