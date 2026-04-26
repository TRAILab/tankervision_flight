#!/usr/bin/env python3
"""
camera_viewer_node.py

Displays Lucid and analog camera feeds in separate resizable windows.
Each window scales its feed to fill the window as you drag it larger.
A "Save" button is drawn in the corner of each window — click it or
press 's' to save both frames to ~/saved_frames/.

Also publishes a ROS save trigger so another node can save its own image
internally when Save is pressed.

Topics:
  Lucid  → /cam0/image_raw
  Analog → /cam1/image_raw
  Save trigger pub → /save_images_trigger

Controls:
  s / click Save  - save frames + publish trigger
  a               - toggle ExposureAuto On/Off
  z               - toggle GainAuto On/Off
  = / -           - exposure +/- 1000us
  [ / ]           - gain -/+ 1dB
  , / .           - brightness -/+ 5
  9 / 0           - gamma -/+ 0.1
  m               - toggle Mean/Median algorithm
  q / ESC         - quit
"""

import os
import re
import json
import threading
from datetime import datetime

os.environ.setdefault('QT_QPA_FONTDIR', '/usr/share/fonts')

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# ── Topics ────────────────────────────────────────────────────────────────────
LUCID_TOPIC        = '/cam0/image_raw'
ANALOG_TOPIC       = '/cam1/image_raw'
SAVE_TRIGGER_TOPIC = '/save_images_trigger'

# ── QoS ───────────────────────────────────────────────────────────────────────
_LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
_TRIGGER_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# ── Save button geometry ──────────────────────────────────────────────────────
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
    try:
        _, _, win_w, win_h = cv2.getWindowImageRect(win_name)
    except Exception:
        return frame
    if win_w <= 0 or win_h <= 0:
        return frame
    fh, fw = frame.shape[:2]
    scale = min(win_w / fw, win_h / fh)
    new_w, new_h = int(fw * scale), int(fh * scale)
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.zeros((win_h, win_w, 3), dtype=np.uint8)
    x_off = (win_w - new_w) // 2
    y_off = (win_h - new_h) // 2
    canvas[y_off:y_off + new_h, x_off:x_off + new_w] = resized
    return canvas


def _draw_save_button(img: np.ndarray, pressed: bool = False) -> np.ndarray:
    out = img.copy()
    color = (0, 255, 80) if pressed else (0, 180, 0)
    cv2.rectangle(out, (BTN_X, BTN_Y), (BTN_X + BTN_W, BTN_Y + BTN_H), color, -1)
    cv2.rectangle(out, (BTN_X, BTN_Y), (BTN_X + BTN_W, BTN_Y + BTN_H), (255, 255, 255), 2)
    cv2.putText(out, '[ Save ]', (BTN_X + 8, BTN_Y + 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return out


def _placeholder(label: str) -> np.ndarray:
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, f'Waiting for {label}...', (20, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 180), 2)
    return img


def _label(img: np.ndarray, text: str) -> np.ndarray:
    out = img.copy()
    h = out.shape[0]
    y = max(h - 15, 20)
    cv2.putText(out, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return out


def _in_button(x: int, y: int) -> bool:
    return BTN_X <= x <= BTN_X + BTN_W and BTN_Y <= y <= BTN_Y + BTN_H


# ── Parameter helpers ─────────────────────────────────────────────────────────

def _p_double(name, value):
    p = Parameter()
    p.name = name
    p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
    return p

def _p_int(name, value):
    p = Parameter()
    p.name = name
    p.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(value))
    return p

def _p_bool(name, value):
    p = Parameter()
    p.name = name
    p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(value))
    return p

def _p_string(name, value):
    p = Parameter()
    p.name = name
    p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(value))
    return p


# ── Overlays ──────────────────────────────────────────────────────────────────

def _draw_lucid_params(img: np.ndarray, diag: dict) -> np.ndarray:
    out = img.copy()
    lines = [
        f"ExposureAuto: {diag.get('exposure_auto', '?')}",
        f"ExposureTime(us): {diag.get('exposure_time', '?')}",
        f"GainAuto: {diag.get('gain_auto', '?')}",
        f"Gain(dB): {diag.get('gain', '?')}",
        f"TargetBrightness: {diag.get('target_brightness', '?')}",
        f"Algorithm: {diag.get('exposure_auto_algorithm', '?')}",
        f"Damping: {diag.get('exposure_auto_damping', '?')}",
        f"AOI: {diag.get('auto_exposure_aoi_enable', '?')} "
        f"{diag.get('auto_exposure_aoi_width', '?')}x{diag.get('auto_exposure_aoi_height', '?')} "
        f"+({diag.get('auto_exposure_aoi_offset_x', '?')},{diag.get('auto_exposure_aoi_offset_y', '?')})",
        f"Gamma: {diag.get('gamma', '?')}",
        f"FPS Ctrl: {diag.get('acquisition_frame_rate_enable', '?')} / {diag.get('acquisition_frame_rate', '?')}",
        f"Mean/Median: {diag.get('calculated_mean', '?')} / {diag.get('calculated_median', '?')}",
    ]
    y = BTN_Y + BTN_H + 25
    for line in lines:
        cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        y += 22
    return out


def _draw_controls(img: np.ndarray) -> np.ndarray:
    out = img.copy()
    lines = [
        '── Controls ──────────────',
        's        save frames',
        'a        toggle ExposureAuto',
        'z        toggle GainAuto',
        '= / -    exposure +/- 1000us',
        '[ / ]    gain -/+ 1dB',
        ', / .    brightness -/+ 5',
        '9 / 0    gamma -/+ 0.1',
        'm        toggle Mean/Median',
        'q / ESC  quit',
    ]
    font       = cv2.FONT_HERSHEY_SIMPLEX
    scale      = 0.45
    thickness  = 1
    line_h     = 18
    padding    = 10

    max_w = max(cv2.getTextSize(l, font, scale, thickness)[0][0] for l in lines)
    x = out.shape[1] - max_w - padding - 5
    y_start = out.shape[0] - len(lines) * line_h - padding

    cv2.rectangle(out,
                  (x - 5, y_start - 15),
                  (out.shape[1] - padding + 5, out.shape[0] - padding + 5),
                  (0, 0, 0), -1)
    cv2.rectangle(out,
                  (x - 5, y_start - 15),
                  (out.shape[1] - padding + 5, out.shape[0] - padding + 5),
                  (80, 80, 80), 1)

    y = y_start
    for line in lines:
        color = (100, 200, 255) if line.startswith('──') else (200, 200, 200)
        cv2.putText(out, line, (x, y), font, scale, color, thickness)
        y += line_h

    return out


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

        self.declare_parameter('show_lucid',         True)
        self.declare_parameter('show_analog',         True)
        self.declare_parameter('save_dir',            os.path.expanduser('~/saved_frames'))
        self.declare_parameter('save_trigger_topic',  SAVE_TRIGGER_TOPIC)

        self._show_lucid         = self.get_parameter('show_lucid').value
        self._show_analog        = self.get_parameter('show_analog').value
        self._save_dir           = self.get_parameter('save_dir').value
        self._save_trigger_topic = self.get_parameter('save_trigger_topic').value

        if not self._show_lucid and not self._show_analog:
            raise RuntimeError("At least one of 'show_lucid' or 'show_analog' must be True")

        self._lock         = threading.Lock()
        self._frame_lucid  = None
        self._frame_analog = None
        self._lucid_diag   = {}

        # Session path from status_node — update save_dir when received
        self.create_subscription(
            String, '/tankervision/session_path', self._session_cb, _LATCHED_QOS)

        os.makedirs(self._save_dir, exist_ok=True)

        if self._show_lucid:
            self.create_subscription(Image, LUCID_TOPIC, self._lucid_cb, _BEST_EFFORT_QOS)
            self.get_logger().info(f'Subscribed to {LUCID_TOPIC} (Lucid)')

        if self._show_analog:
            self.create_subscription(Image, ANALOG_TOPIC, self._analog_cb, _BEST_EFFORT_QOS)
            self.get_logger().info(f'Subscribed to {ANALOG_TOPIC} (Analog)')

        self._save_trigger_pub = self.create_publisher(Empty, self._save_trigger_topic, _TRIGGER_QOS)
        self._param_client     = self.create_client(SetParameters, '/arena_camera_node/set_parameters')

        self.create_subscription(String, '/camera/lucid_diagnostics', self._diag_cb, 10)

        self.get_logger().info(f"Publishing save triggers on {self._save_trigger_topic}")
        self.get_logger().info("Controls shown on Lucid window. 'q'/ESC to quit.")

    # ── Callbacks ─────────────────────────────────────────────────────────────

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

    def _diag_cb(self, msg: String):
        try:
            self._lucid_diag = json.loads(msg.data)
        except Exception:
            # Salvage individual key:value pairs from malformed JSON
            result = {}
            for m in re.finditer(r'"(\w+)":\s*(".*?"|null|true|false|-?[\d.]+)', msg.data):
                key, val = m.group(1), m.group(2)
                try:
                    result[key] = json.loads(val)
                except Exception:
                    result[key] = val.strip('"')
            if result:
                self._lucid_diag = result

    # ── Parameter control ─────────────────────────────────────────────────────

    def _set_remote_params(self, params):
        if not self._param_client.service_is_ready():
            self.get_logger().warn('arena_camera_node parameter service not ready')
            return
        req = SetParameters.Request()
        req.parameters = params
        self._param_client.call_async(req)

    # ── Frame access ──────────────────────────────────────────────────────────

    def get_frames(self):
        with self._lock:
            lucid  = self._frame_lucid.copy()  if self._frame_lucid  is not None else None
            analog = self._frame_analog.copy() if self._frame_analog is not None else None
        return lucid, analog

    # ── Save ──────────────────────────────────────────────────────────────────

    def _session_cb(self, msg):
        """Update save_dir when status_node publishes session path."""
        session_dir = msg.data
        new_save_dir = os.path.join(session_dir, 'saved_frames')
        if new_save_dir != self._save_dir:
            self._save_dir = new_save_dir
            os.makedirs(self._save_dir, exist_ok=True)
            self.get_logger().info(f'Save dir updated to: {self._save_dir}')

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
            self.get_logger().info(f'Saved locally: {p}')

    def publish_save_trigger(self):
        self._save_trigger_pub.publish(Empty())
        self.get_logger().info(f'Published save trigger on {self._save_trigger_topic}')

    def handle_save_action(self, lucid, analog):
        self.save_frames(lucid, analog)
        self.publish_save_trigger()


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

    save_flash = 0

    try:
        while rclpy.ok():
            lucid, analog = node.get_frames()

            do_save = False
            for ms in mouse_states.values():
                if ms.clicked:
                    ms.clicked = False
                    do_save = True

            if do_save:
                node.handle_save_action(lucid, analog)
                save_flash = 10

            pressed = save_flash > 0
            if save_flash > 0:
                save_flash -= 1

            # Lucid — fit first, then all overlays at display resolution
            if node._show_lucid:
                raw   = lucid if lucid is not None else _placeholder('Lucid')
                frame = _fit_to_window(raw, 'Lucid Camera')
                frame = _draw_save_button(frame, pressed)
                frame = _draw_lucid_params(frame, node._lucid_diag)
                frame = _draw_controls(frame)
                frame = _label(frame, 'Lucid')
                cv2.imshow('Lucid Camera', frame)

            # Analog — fit first, then overlays at display resolution
            if node._show_analog:
                raw   = analog if analog is not None else _placeholder('Analog')
                frame = _fit_to_window(raw, 'Analog Camera')
                frame = _draw_save_button(frame, pressed)
                frame = _label(frame, 'Analog')
                cv2.imshow('Analog Camera', frame)

            key = cv2.waitKey(30) & 0xFF

            if key == ord('s'):
                node.handle_save_action(lucid, analog)
                save_flash = 10

            elif key == ord('a'):
                current = str(node._lucid_diag.get('exposure_auto', 'Off'))
                new_val = 'Continuous' if current == 'Off' else 'Off'
                node._set_remote_params([_p_string('exposure_auto', new_val)])

            elif key == ord('z'):
                current = str(node._lucid_diag.get('gain_auto', 'Off'))
                new_val = 'Continuous' if current == 'Off' else 'Off'
                node._set_remote_params([_p_string('gain_auto', new_val)])

            elif key == ord('='):
                cur = float(node._lucid_diag.get('exposure_time', 20000.0))
                node._set_remote_params([_p_double('exposure_time', cur + 1000.0)])

            elif key == ord('-'):
                cur = float(node._lucid_diag.get('exposure_time', 20000.0))
                node._set_remote_params([_p_double('exposure_time', max(100.0, cur - 1000.0))])

            elif key == ord(']'):
                cur = float(node._lucid_diag.get('gain', 0.0))
                node._set_remote_params([_p_double('gain', cur + 1.0)])

            elif key == ord('['):
                cur = float(node._lucid_diag.get('gain', 0.0))
                node._set_remote_params([_p_double('gain', max(0.0, cur - 1.0))])

            elif key == ord('.'):
                cur = node._lucid_diag.get('target_brightness', 128)
                cur = int(cur) if cur is not None else 128
                new_val = min(255, cur + 5)
                node.get_logger().info(f'Setting target_brightness to {new_val}')
                node._set_remote_params([_p_int('target_brightness', new_val)])

            elif key == ord(','):
                cur = node._lucid_diag.get('target_brightness', 128)
                cur = int(cur) if cur is not None else 128
                new_val = max(0, cur - 5)
                node.get_logger().info(f'Setting target_brightness to {new_val}')
                node._set_remote_params([_p_int('target_brightness', new_val)])

            elif key == ord('0'):
                cur = float(node._lucid_diag.get('gamma', 1.0))
                node._set_remote_params([_p_double('gamma', round(min(3.0, cur + 0.1), 2))])

            elif key == ord('9'):
                cur = float(node._lucid_diag.get('gamma', 1.0))
                node._set_remote_params([_p_double('gamma', round(max(0.1, cur - 0.1), 2))])

            elif key == ord('m'):
                current = str(node._lucid_diag.get('exposure_auto_algorithm', 'Mean'))
                new_val = 'Median' if current == 'Mean' else 'Mean'
                node._set_remote_params([_p_string('exposure_auto_algorithm', new_val)])

            elif key in (ord('q'), 27):
                break

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()