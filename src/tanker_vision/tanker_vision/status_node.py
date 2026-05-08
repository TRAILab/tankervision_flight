import io
import sys
import os
import math
import socket
import shutil
import yaml
import subprocess
import threading
import logging

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

from rcl_interfaces.msg import Log
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Imu, Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from .send_email import send_email, EmailError

# ─── QoS profiles ─────────────────────────────────────────────────────────────
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

# ─── Logging setup ─────────────────────────────────────────────────────────────
_log_stream = io.StringIO()
_formatter = logging.Formatter(
    '[%(asctime)s] %(levelname)s %(name)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
)

_stream_handler = logging.StreamHandler(_log_stream)
_stream_handler.setLevel(logging.INFO)
_stream_handler.setFormatter(_formatter)

_console_handler = logging.StreamHandler(sys.stdout)
_console_handler.setLevel(logging.INFO)
_console_handler.setFormatter(_formatter)

_py_logger = logging.getLogger('status_node')
_py_logger.setLevel(logging.INFO)
_py_logger.handlers.clear()
_py_logger.addHandler(_console_handler)
_py_logger.addHandler(_stream_handler)
_py_logger.propagate = False


# ─── Config helpers ─────────────────────────────────────────────────────────────
def _deep_merge(base: dict, override: dict) -> dict:
    result = dict(base)
    for k, v in override.items():
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = _deep_merge(result[k], v)
        else:
            result[k] = v
    return result


def _load_config(node) -> dict:
    """Load and merge global tankervision.yaml + unit-specific yaml."""
    cfg = {}
    for param in ('config', 'unit_config'):
        try:
            path = node.get_parameter(param).get_parameter_value().string_value
        except Exception:
            continue
        if path and os.path.exists(path):
            try:
                with open(path) as f:
                    data = yaml.safe_load(f) or {}
                cfg = _deep_merge(cfg, data)
            except Exception as e:
                _py_logger.error(f'Failed to load config {path}: {e}')
    return cfg


# ─── System utilities ───────────────────────────────────────────────────────────
def chrony_has_pps() -> bool:
    try:
        result = subprocess.run(
            ['chronyc', 'sources'],
            capture_output=True, text=True, check=True,
        )
    except subprocess.CalledProcessError:
        return False
    if '506' in result.stdout:
        return False
    parsing = False
    for line in result.stdout.splitlines():
        if line.strip().startswith('===='):
            parsing = True
            continue
        if not parsing:
            continue
        if 'PPS' in line:
            cols = line.split(None, 8)
            if len(cols) >= 5:
                try:
                    reach = int(cols[4], 8)
                    return (reach & 0x01) != 0
                except ValueError:
                    pass
    return False


def has_internet() -> bool:
    try:
        socket.create_connection(('www.google.com', 80), timeout=1.0)
        return True
    except OSError:
        return False


def get_free_space_percentage(path: str = '/') -> str:
    usage = shutil.disk_usage(path)
    if usage.total == 0:
        return '00'
    percent = int((usage.free / usage.total) * 100)
    return f'{min(percent, 99):02d}'


def is_disk_mounted(mount_point: str = '/mnt/storage') -> bool:
    return os.path.ismount(mount_point)


# ─── State handler ──────────────────────────────────────────────────────────────
class StateHandler:
    def __init__(self, name: str, ok_state: str, no_heartbeat_state: str):
        self.name = name
        self.state = None
        self.last_heartbeat = None
        self.ok_state = ok_state
        self.no_heartbeat_state = no_heartbeat_state

    def update_heartbeat(self, now, new_state: str = None):
        self.last_heartbeat = now
        self._set_state(new_state or self.ok_state)

    def check_heartbeat_timeout(self, now, timeout_sec: float):
        if (self.last_heartbeat is None or
                (now - self.last_heartbeat).nanoseconds / 1e9 > timeout_sec):
            self._set_state(self.no_heartbeat_state)

    def _set_state(self, new_state: str):
        if self.state != new_state:
            self.state = new_state
            _py_logger.info(f'{self.name} state → {new_state}')


# ─── StatusNode ─────────────────────────────────────────────────────────────────
class StatusNode(Node):
    def __init__(self):
        super().__init__('status_node')

        self.declare_parameter('config', '')
        self.declare_parameter('unit_config', '')

        self._cfg          = _load_config(self)
        self._notif        = self._cfg.get('notifications', {})
        self._unit         = self._cfg.get('unit', {})
        self._mode         = self._cfg.get('mode', 'testing')
        self._storage_root = self._cfg.get('storage', {}).get('root', '/mnt/storage')
        self._email_creds  = dict(
            username  = self._notif.get('gmail_user', ''),
            password  = self._notif.get('gmail_app_password', ''),
            sender    = self._notif.get('gmail_user', ''),
            recipient = self._notif.get('email_to', ''),
        )

        self._executor = ThreadPoolExecutor(max_workers=4)

        self.is_in_air          = False
        self.send_startup_email = True
        self.send_landing_email = False
        self.camera_time_sync_state = None
        self.internet_state     = None
        self.time_sync_state    = None
        self.velocity_msg_count = 0

        self._session_path      = None
        self._flight_log_fh     = None
        self._maxvis_ever_active = False
        self._maxvis_last_check  = 0.0
        self._maxvis_device      = self._cfg.get('analog_camera', {}).get('device', '/dev/video0')
        self._record_count = 0

        # Hardware trigger mode from config
        _ht = self._cfg.get('lucid_camera', {}).get('hardware_trigger', False)
        self._hw_trigger_mode = str(_ht).lower()   # 'false' | 'true' | 'on_pps'
        self._pps_trigger_done = False

        # ─── State handlers ──────────────────────────────────────────────
        self.imu       = StateHandler('IMU',       'IMU_OK',              'IMU_NO_HEARTBEAT')
        self.camera    = StateHandler('CAMERA',    'CAMERA_RECEIVING',    'CAMERA_NO_HEARTBEAT')
        self.recording = StateHandler('RECORDING', 'RECORDING_RECORDING', 'RECORDING_NO_HEARTBEAT')

        # ─── Session path publisher (latched) ────────────────────────────
        self._session_path_pub = self.create_publisher(
            String, '/tankervision/session_path', _LATCHED_QOS)

        # ─── Subscriptions ───────────────────────────────────────────────
        self.create_subscription(Imu,    '/im19/imu',            self._imu_heartbeat_cb,       10)
        self.create_subscription(String, 'camera_heartbeat',     self._camera_heartbeat_cb,    10)
        self.create_subscription(String, 'record_data/status',   self._recording_heartbeat_cb, 10)
        self.create_subscription(String, '/camera/record_mode',  self._record_mode_cb,         10)
        self.create_subscription(Log,    '/rosout',              self._rosout_cb,              100)
        # MaxVis: best-effort, depth=1 — only used to detect if analog signal is present
        self.create_subscription(Image, '/cam1/image_raw', self._maxvis_cb, _BEST_EFFORT_QOS)
        # TODO: subscribe to velocity topic once GNSS velocity source is available
        # (xsens /filter/velocity is gone; IM19 does not publish a fused velocity)

        # ─── Timers ──────────────────────────────────────────────────────
        self.create_timer(1.25, self._check_camera_heartbeat)
        self.create_timer(1.0,  self._check_imu_heartbeat)
        self.create_timer(1.25, self._check_recording_heartbeat)
        self.create_timer(5.0,  self._check_internet)
        self.create_timer(10.0, self._check_time_sync)
        self._session_timer = self.create_timer(5.0, self._try_create_session)

        # Attempt session folder creation immediately; timer retries on failure
        self._try_create_session()

        _py_logger.info(
            f'StatusNode started | unit={self._unit.get("name", "?")} '
            f'plane={self._unit.get("plane_number", "?")} mode={self._mode}'
        )

    # ─── Session folder ───────────────────────────────────────────────────────
    def _try_create_session(self):
        if self._session_path is not None:
            self._session_timer.cancel()
            return

        if datetime.now().year < 2020:
            _py_logger.info('Waiting for valid clock (currently pre-2020) before creating session folder...')
            return

        if not chrony_has_pps():
            _py_logger.warning('No GPS PPS — session clock from NTP/cellular only')

        if not is_disk_mounted(self._storage_root):
            _py_logger.error(f'Storage not mounted at {self._storage_root} — retrying...')
            return

        now = datetime.now()
        session_path = os.path.join(
            self._storage_root,
            now.strftime('%Y-%m-%d'),
            now.strftime('session_%H-%M-%S'),
        )

        try:
            os.makedirs(session_path, exist_ok=True)
            os.makedirs(os.path.join(session_path, 'saved_frames'), exist_ok=True)
        except OSError as e:
            _py_logger.error(f'Failed to create session folder: {e}')
            return

        self._session_path = session_path
        self._session_timer.cancel()

        # Add flight log file handler
        log_path = os.path.join(session_path, 'flight_log.txt')
        fh = logging.FileHandler(log_path)
        fh.setLevel(logging.INFO)
        fh.setFormatter(_formatter)
        _py_logger.addHandler(fh)
        self._flight_log_fh = fh

        _py_logger.info(f'=== SESSION STARTED: {session_path} ===')

        # Write session path to a well-known file so non-ROS scripts (gnss, imu) can find it
        try:
            with open('/tmp/tankervision_session_path', 'w') as f:
                f.write(session_path)
        except Exception as e:
            _py_logger.warning(f'Could not write session path file: {e}')

        # Latched publish — late-starting nodes will still receive this
        msg = String()
        msg.data = session_path
        self._session_path_pub.publish(msg)

    # ─── Heartbeat callbacks ──────────────────────────────────────────────────
    def _imu_heartbeat_cb(self, _msg: Imu):
        self.imu.update_heartbeat(self.get_clock().now())

    def _camera_heartbeat_cb(self, msg: String):
        now  = self.get_clock().now()
        data = msg.data.lower()
        state = 'CAMERA_RECEIVING'
        if 'timeout' in data:
            state = 'CAMERA_TIMEOUT'
        elif 'error' in data or 'exception' in data:
            state = 'CAMERA_ERROR'
        self.camera.update_heartbeat(now, state)

        sync_state = 'CAMERA_TIME_SYNCED' if 'slave' in data else 'CAMERA_TIME_NOT_SYNCED'
        if sync_state != self.camera_time_sync_state:
            self.camera_time_sync_state = sync_state
            _py_logger.info(f'Camera time sync: {sync_state}')

    def _recording_heartbeat_cb(self, msg: String):
        now = self.get_clock().now()
        low = msg.data.lower()
        if 'recording' in low:
            st = 'RECORDING_RECORDING'
        elif 'scanning' in low:
            st = 'RECORDING_SCANNING'
        else:
            return
        self.recording.update_heartbeat(now, st)

    def _check_imu_heartbeat(self):
        self.imu.check_heartbeat_timeout(self.get_clock().now(), 2.0)

    def _check_camera_heartbeat(self):
        self.camera.check_heartbeat_timeout(self.get_clock().now(), 3.0)

    def _check_recording_heartbeat(self):
        self.recording.check_heartbeat_timeout(self.get_clock().now(), 2.0)

    # ─── /rosout and trigger logging ─────────────────────────────────────────
    def _rosout_cb(self, msg: Log):
        if self._flight_log_fh is None:
            return
        level_map = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
        if msg.level >= 30:  # WARN and above from all nodes → flight log
            line = f'[rosout][{level_map.get(msg.level, "?")}][{msg.name}] {msg.msg}\n'
            self._flight_log_fh.stream.write(line)
            self._flight_log_fh.stream.flush()

    def _maxvis_cb(self, _msg: Image):
        if self._maxvis_ever_active:
            return
        import time
        now = time.monotonic()
        if now - self._maxvis_last_check < 1.0:
            return
        self._maxvis_last_check = now
        try:
            out = subprocess.check_output(
                ['v4l2-ctl', f'--device={self._maxvis_device}', '--get-input'],
                stderr=subprocess.DEVNULL, timeout=2,
            ).decode()
            if 'no signal' not in out.lower():
                self._maxvis_ever_active = True
                _py_logger.info('MaxVis analog signal detected')
        except Exception:
            pass

    def _record_mode_cb(self, msg: String):
        if msg.data.strip().lower() == 'record':
            self._record_count += 1
            _py_logger.info(f'Recording started #{self._record_count}')

    # ─── Internet & time sync ─────────────────────────────────────────────────
    def _check_internet(self):
        threading.Thread(target=self._check_internet_bg, daemon=True).start()

    def _check_internet_bg(self):
        connected = has_internet()
        state = 'INTERNET_OK' if connected else 'INTERNET_NO_CONNECTION'
        if state != self.internet_state:
            self.internet_state = state
            _py_logger.info(f'Internet: {state}')

            name  = self._unit.get('name', 'unit')
            plane = self._unit.get('plane_number', '?')

            if connected and self.send_landing_email:
                self.send_landing_email = False
                self._executor.submit(self._send_landing_email_bg)
            elif connected and self.send_startup_email and self._session_path:
                self.send_startup_email = False
                body = (
                    f'TankerVision is online.\n'
                    f'Unit: {name} | Plane: {plane} | Mode: {self._mode}\n'
                    f'Session: {self._session_path}'
                )
                self._executor.submit(self._send_email, self._email_subject('Online'), body)

    def _check_time_sync(self):
        threading.Thread(target=self._check_time_sync_bg, daemon=True).start()

    def _check_time_sync_bg(self):
        pps_ok = chrony_has_pps()
        state  = 'TIME_SYNC_PPS' if pps_ok else 'TIME_SYNC_NO_PPS'
        if state != self.time_sync_state:
            self.time_sync_state = state
            _py_logger.info(f'Time sync: {state}')
            if pps_ok and self._hw_trigger_mode == 'on_pps' and not self._pps_trigger_done:
                self._pps_trigger_done = True
                threading.Thread(target=self._restart_arena_with_hardware_trigger, daemon=True).start()


    # ─── Restart Camera on PPS ────────────────────────────────────────────────
    def _restart_arena_with_hardware_trigger(self):
        _py_logger.info('PPS acquired — restarting arena_camera_node with hardware trigger')
        try:
            subprocess.run(
                ['pkill', '-f', 'arena_camera_node'],
                check=False, timeout=5
            )
            import time; time.sleep(2)  # let node die cleanly
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'
            subprocess.Popen(
                [
                    'bash', '-c',
                    'source /opt/ros/humble/setup.bash && '
                    f'source {os.path.expanduser("~")}/tankervision_flight/install/setup.bash && '
                    'ros2 run arena_camera_node start '
                    '--ros-args '
                    '-r /arena_camera_node/images:=/cam0/image_raw '
                    '-p hardware_trigger:=true '
                    f'-p width:={self._cfg.get("lucid_camera", {}).get("width", 5320)} '
                    f'-p height:={self._cfg.get("lucid_camera", {}).get("height", 4600)} '
                    f'-p pixelformat:={self._cfg.get("lucid_camera", {}).get("pixelformat", "bayer_rggb16")} '
                    f'-p config:={self.get_parameter("config").get_parameter_value().string_value} '
                    f'-p unit_config:={self.get_parameter("unit_config").get_parameter_value().string_value}'
                ],
                env=env,
            )
            _py_logger.info('arena_camera_node restarted with hardware_trigger=true')
        except Exception as e:
            _py_logger.error(f'Failed to restart arena_camera_node: {e}')

    # ─── Flight detection ─────────────────────────────────────────────────────
    def _takeoff_procedure(self, mag: float):
        name  = self._unit.get('name', 'unit')
        plane = self._unit.get('plane_number', '?')

        logs = _log_stream.getvalue()
        _log_stream.truncate(0)
        _log_stream.seek(0)

        try:
            du_output = subprocess.check_output(
                ['du', '-sh', self._storage_root],
                text=True, stderr=subprocess.STDOUT,
            )
        except subprocess.CalledProcessError as e:
            du_output = f'du error: {e.output or e}'

        body = (
            f'Takeoff detected for {name} ({plane}).\n'
            f'Velocity: {mag:.2f} m/s\n'
            f'Session: {self._session_path}\n\n'
            f'=== NODE LOGS ===\n{logs}\n\n'
            f'=== STORAGE USAGE ===\n{du_output}'
        )
        self._send_email(self._email_subject('Takeoff'), body)

    def velocity_callback(self, msg):
        self.velocity_msg_count += 1
        if self.velocity_msg_count % 100:
            return
        mag = math.sqrt(msg.vector.x**2 + msg.vector.y**2 + msg.vector.z**2)

        if mag > 14.0 and not self.is_in_air:
            self.is_in_air = True
            _py_logger.info(f'Takeoff detected: {mag:.2f} m/s')
            self._executor.submit(self._takeoff_procedure, mag)
        elif mag < 1.0 and self.is_in_air:
            self.is_in_air = False
            _py_logger.info(f'Landing detected: {mag:.2f} m/s')
            self.send_landing_email = True

    # ─── Email helpers ────────────────────────────────────────────────────────
    def _email_subject(self, event: str) -> str:
        name  = self._unit.get('name', 'unit')
        plane = self._unit.get('plane_number', '?')
        return f'TankerVision {event} — {name} ({plane})'

    def _send_email(self, subject: str, body: str):
        try:
            send_email(subject, body, **self._email_creds)
            _py_logger.info(f'Email sent: {subject}')
        except EmailError as e:
            _py_logger.error(f'Email failed ({subject}): {e}')

    def _send_landing_email_bg(self):
        logs = _log_stream.getvalue()
        _log_stream.truncate(0)
        _log_stream.seek(0)

        # Count raw frames saved in this session
        raw_count = 0
        if self._session_path and os.path.isdir(self._session_path):
            raw_count = sum(
                1 for f in os.listdir(self._session_path) if f.endswith('.raw')
            )

        try:
            du_output = subprocess.check_output(
                ['du', '-sh', self._storage_root],
                text=True, stderr=subprocess.STDOUT,
            )
        except subprocess.CalledProcessError as e:
            du_output = f'du error: {e.output or e}'

        maxvis_str = 'YES — signal detected this session' if self._maxvis_ever_active else 'NO — no signal detected'

        body = (
            f'The plane has landed.\n\n'
            f'=== SESSION SUMMARY ===\n'
            f'Session folder : {self._session_path}\n'
            f'Raw images saved: {raw_count}\n'
            f'Recordings      : {self._record_count}\n'
            f'MaxVis active   : {maxvis_str}\n\n'
            f'=== NODE LOGS ===\n{logs}\n\n'
            f'=== STORAGE USAGE ===\n{du_output}'
        )
        self._send_email(self._email_subject('Landing Alert'), body)


def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
