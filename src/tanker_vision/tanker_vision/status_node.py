#!/usr/bin/env python3
"""
status_node.py — TankerVision flight status monitor.

Creates the session folder at startup and publishes its path on
/tankervision/session_path (latched) so all nodes write to the same location.

Session structure:
  /mnt/storage/YYYY-MM-DD/session_HH-MM-SS/
    flight_log.txt
    saved_frames/
    ros_log/          (copied on shutdown)
"""

import os
import re
import shutil
import socket
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import Log
from std_msgs.msg import Empty, String
import yaml


# ── QoS ──────────────────────────────────────────────────────────────────────
_ROSOUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
)
_LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# ── Terminal colours ──────────────────────────────────────────────────────────
GRN  = '\033[0;32m'
YEL  = '\033[1;33m'
CYN  = '\033[0;36m'
BOLD = '\033[1m'
NC   = '\033[0m'


# ── Helpers ───────────────────────────────────────────────────────────────────
def _load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def chrony_synced() -> bool:
    try:
        r = subprocess.run(['chronyc', 'tracking'], capture_output=True, text=True, check=True)
        return 'Reference ID' in r.stdout and '0.0.0.0' not in r.stdout
    except Exception:
        return False


def ptp_master_active() -> bool:
    try:
        r = subprocess.run(['systemctl', 'is-active', 'ptp4l.service'],
                           capture_output=True, text=True)
        if r.stdout.strip() != 'active':
            return False
        logs = subprocess.run(['journalctl', '-u', 'ptp4l.service', '-n', '20', '--no-pager'],
                              capture_output=True, text=True)
        return 'MASTER' in logs.stdout or 'assuming the grand master' in logs.stdout
    except Exception:
        return False


def ptp_slave_locked() -> bool:
    try:
        logs = subprocess.run(
            ['journalctl', '-u', 'tankervision.service', '-n', '50', '--no-pager'],
            capture_output=True, text=True)
        return 'PTP status: Slave' in logs.stdout
    except Exception:
        return False


def maxvis_has_signal(device: str = '/dev/video0') -> bool:
    try:
        r = subprocess.run(['v4l2-ctl', '--device', device, '--get-input'],
                           capture_output=True, text=True, timeout=2)
        return 'no signal' not in r.stdout.lower()
    except Exception:
        return False


def service_active(name: str) -> bool:
    try:
        r = subprocess.run(['systemctl', 'is-active', name],
                           capture_output=True, text=True)
        return r.stdout.strip() == 'active'
    except Exception:
        return False


def ros_node_active(node_name: str) -> bool:
    try:
        r = subprocess.run(['ros2', 'node', 'list'],
                           capture_output=True, text=True, timeout=3)
        return f'/{node_name}' in r.stdout
    except Exception:
        return False


def lucid_connected() -> bool:
    try:
        logs = subprocess.run(
            ['journalctl', '-u', 'tankervision.service', '-n', '100', '--no-pager'],
            capture_output=True, text=True)
        return 'Pixel format' in logs.stdout
    except Exception:
        return False


def has_internet() -> bool:
    try:
        socket.create_connection(('www.google.com', 80), timeout=2.0)
        return True
    except OSError:
        return False


def send_email_msmtp(to: str, subject: str, body: str, msmtprc: str = None) -> bool:
    try:
        cmd = ['msmtp']
        if msmtprc and os.path.exists(msmtprc):
            cmd += ['-C', msmtprc]
        cmd.append(to)
        msg = f'To: {to}\nSubject: {subject}\n\n{body}'
        result = subprocess.run(cmd, input=msg, capture_output=True, text=True, timeout=15)
        return result.returncode == 0
    except Exception:
        return False


def storage_info(path: str):
    try:
        usage = shutil.disk_usage(path)
        free  = usage.free  / (1024 ** 4)
        total = usage.total / (1024 ** 4)
        return round(free, 1), round(total, 1)
    except Exception:
        return None, None


# ── Status node ───────────────────────────────────────────────────────────────
class StatusNode(Node):

    EMAIL_RETRY_SEC = 300.0
    MAXVIS_POLL_SEC = 2.0

    _NODE_PATTERNS = {
        'arena_camera_node': [
            (r'No arena camera.*Waiting',  'LUCID    Waiting for camera...',  'lucid_waiting'),
            (r'PTP status: Slave',         'LUCID    PTP slave locked',        'lucid_ptp_slave'),
            (r'Pixel format',              'LUCID    Connected',               'lucid_connected'),
        ],
        'im19_mems_node': [
            (r'Failed to open serial',     'IMU      Waiting for /dev/im19_mems', 'imu_waiting'),
            (r'reconnect|Connected|open',  'IMU      Connected',               'imu_connected'),
        ],
        'analog_camera': [
            (r'Buffer failure.*busy',      'ANALOG   Device busy on start',   'analog_busy'),
            (r'Starting camera',           'ANALOG   Camera started',          'analog_started'),
        ],
    }

    def __init__(self):
        super().__init__('status_node')

        # ── Load config ───────────────────────────────────────────
        _default_cfg = os.path.join(
            os.path.expanduser('~'),
            'tankervision_flight', 'config', 'tankervision.yaml'
        )
        yaml_path = self.declare_parameter('config', _default_cfg).value
        try:
            cfg = _load_yaml(yaml_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            cfg = {}

        unit_cfg   = cfg.get('unit', {})
        notif_cfg  = cfg.get('notifications', {})
        stor_cfg   = cfg.get('storage', {})
        analog_cfg = cfg.get('analog_camera', {})

        self._unit_name   = unit_cfg.get('name', 'unknown')
        self._plane       = unit_cfg.get('plane_number', 'unknown')
        self._province    = unit_cfg.get('province', 'unknown')
        self._mode        = cfg.get('mode', 'testing')
        self._email_to    = notif_cfg.get('email_to', '')
        self._msmtprc     = notif_cfg.get('msmtprc', os.path.expanduser('~/.msmtprc'))
        self._storage     = stor_cfg.get('root', '/mnt/storage')
        self._v4l2_device = analog_cfg.get('device', '/dev/video0')

        # ── Create session folder ─────────────────────────────────
        now = datetime.now()
        date_str    = now.strftime('%Y-%m-%d')
        session_str = now.strftime('session_%H-%M-%S')
        self._session_dir = os.path.join(self._storage, date_str, session_str)
        os.makedirs(self._session_dir, exist_ok=True)
        os.makedirs(os.path.join(self._session_dir, 'saved_frames'), exist_ok=True)

        # ── State ─────────────────────────────────────────────────
        self._startup_time    = now
        self._email_sent      = False
        self._maxvis_active   = False
        self._trigger_count   = 0
        self._error_count     = 0
        self._maxvis_on_time  = 0.0
        self._maxvis_on_since = None
        self._lock            = threading.Lock()
        self._node_states: dict = {}

        # ── System checks ─────────────────────────────────────────
        self._chrony_ok   = chrony_synced()
        self._ptp_master  = ptp_master_active()
        self._ptp_slave   = ptp_slave_locked()
        self._gnss_ok     = service_active('gnss-record.service')
        self._imu_ok      = ros_node_active('im19_mems_node')
        self._lucid_ok    = lucid_connected()
        free, total       = storage_info(self._storage)
        self._stor_free   = free
        self._stor_total  = total

        # ── Open log file in session folder ───────────────────────
        log_path = os.path.join(self._session_dir, 'flight_log.txt')
        self._log_file    = open(log_path, 'w', buffering=1)
        self._ros_log_ts  = now.strftime('%Y%m%d_%H%M%S')

        # ── Write header and banner ───────────────────────────────
        self._write_header()
        self._write_banner_to_log()
        self._print_banner()

        # ── Publishers ────────────────────────────────────────────
        # Latched session path — new subscribers get it immediately
        self._session_pub = self.create_publisher(
            String, '/tankervision/session_path', _LATCHED_QOS)

        # ── Publish session path ──────────────────────────────────
        msg = String()
        msg.data = self._session_dir
        self._session_pub.publish(msg)
        self._log(f'STATUS   session: {self._session_dir}')

        # ── Subscriptions ─────────────────────────────────────────
        self.create_subscription(Log,   '/rosout',              self._rosout_cb,  _ROSOUT_QOS)
        self.create_subscription(Empty, '/save_images_trigger', self._trigger_cb, 10)

        # ── Timers ────────────────────────────────────────────────
        self.create_timer(self.MAXVIS_POLL_SEC,  self._poll_maxvis)
        self.create_timer(self.EMAIL_RETRY_SEC,  self._retry_email)

        threading.Thread(target=self._try_send_startup_email, daemon=True).start()

        self._log('STATUS   online')

    # ── Banner ────────────────────────────────────────────────────────────────
    def _banner_rows(self) -> list:
        stor_text = (f'{self._stor_free}TB free / {self._stor_total}TB'
                     if self._stor_total else 'not mounted')
        return [
            ('mergerfs',   self._stor_total is not None, stor_text,           'not mounted'),
            ('chrony',     self._chrony_ok,              'synced',            'not synced'),
            ('ptp master', self._ptp_master,             'active',            'not active'),
            ('ptp slave',  self._ptp_slave,              'locked',            'uncalibrated'),
            ('GNSS',       self._gnss_ok,                'recording',         'not running'),
            ('IMU',        self._imu_ok,                 'running',           'not running'),
            ('Lucid',      self._lucid_ok,               'connected',         'waiting...'),
            ('MaxVis',     False,                        'ON',                'offline'),
            ('email',      self._email_sent,             'sent',              'pending...'),
        ]

    def _print_banner(self):
        ts  = self._startup_time.strftime('%Y-%m-%d %H:%M:%S')
        div = '═' * 52
        print(f'\n{BOLD}{CYN}{div}{NC}')
        print(f'{BOLD}  TankerVision — {self._unit_name} ({self._plane}){NC}')
        print(f'  {self._province} | Mode: {self._mode}')
        print(f'  {ts}')
        print(f'  Session: {self._session_dir}')
        print(f'{BOLD}{CYN}{div}{NC}')
        for label, ok, ok_text, fail_text in self._banner_rows():
            dot  = f'{GRN}●{NC}' if ok else f'{YEL}○{NC}'
            text = f'{GRN}{ok_text}{NC}' if ok else f'{YEL}{fail_text}{NC}'
            print(f'  {dot}  {label:<14}{text}')
        print(f'{BOLD}{CYN}{div}{NC}\n')

    def _write_banner_to_log(self):
        ts  = self._startup_time.strftime('%Y-%m-%d %H:%M:%S')
        div = '─' * 52
        lines = [
            div,
            f'  TankerVision — {self._unit_name} ({self._plane})',
            f'  {self._province} | Mode: {self._mode}',
            f'  {ts}',
            f'  Session: {self._session_dir}',
            div,
        ]
        for label, ok, ok_text, fail_text in self._banner_rows():
            dot  = '●' if ok else '○'
            text = ok_text if ok else fail_text
            lines.append(f'  {dot}  {label:<14}{text}')
        lines.append(div)
        lines.append('')
        with self._lock:
            self._log_file.write('\n'.join(lines) + '\n')

    # ── Log helpers ───────────────────────────────────────────────────────────
    def _ts(self) -> str:
        return datetime.now().strftime('%H:%M:%S')

    def _log(self, msg: str):
        line = f'[{self._ts()}] {msg}\n'
        with self._lock:
            self._log_file.write(line)

    def _write_header(self):
        ts = self._startup_time.strftime('%Y-%m-%d %H:%M:%S')
        lines = [
            '=' * 72,
            'TANKERVISION FLIGHT LOG',
            '=' * 72,
            f'Unit:       {self._unit_name}',
            f'Plane:      {self._plane}',
            f'Province:   {self._province}',
            f'Start time: {ts}',
            f'Mode:       {self._mode}',
            f'Session:    {self._session_dir}',
            f'Chrony:     {"synced"     if self._chrony_ok  else "NOT SYNCED"}',
            f'PTP master: {"active"     if self._ptp_master else "not active"}',
            f'PTP slave:  {"locked"     if self._ptp_slave  else "uncalibrated"}',
            f'GNSS:       {"recording"  if self._gnss_ok    else "not running"}',
            f'IMU:        {"running"    if self._imu_ok     else "not running"}',
            f'Lucid:      {"connected"  if self._lucid_ok   else "waiting"}',
            '=' * 72,
            '',
        ]
        with self._lock:
            self._log_file.write('\n'.join(lines) + '\n')

    def _write_footer(self):
        end_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        duration = datetime.now() - self._startup_time
        h, rem   = divmod(int(duration.total_seconds()), 3600)
        m, s     = divmod(rem, 60)
        if self._maxvis_on_since is not None:
            self._maxvis_on_time += time.time() - self._maxvis_on_since
        mv_h, mv_rem = divmod(int(self._maxvis_on_time), 3600)
        mv_m, mv_s   = divmod(mv_rem, 60)
        lines = [
            '',
            '=' * 72,
            f'Session end:    {end_time}',
            f'Duration:       {h:02d}:{m:02d}:{s:02d}',
            f'Triggers:       {self._trigger_count}',
            f'MaxVis on time: {mv_h:02d}:{mv_m:02d}:{mv_s:02d}',
            f'Errors logged:  {self._error_count}',
            '=' * 72,
        ]
        with self._lock:
            self._log_file.write('\n'.join(lines) + '\n')

    # ── /rosout ───────────────────────────────────────────────────────────────
    def _rosout_cb(self, msg: Log):
        if msg.name == 'status_node':
            return
        patterns = self._NODE_PATTERNS.get(msg.name, [])
        for pattern, label, state_key in patterns:
            if re.search(pattern, msg.msg):
                if not self._node_states.get(state_key):
                    self._node_states[state_key] = True
                    self._log(label)
                    if state_key == 'lucid_connected':
                        self._lucid_ok = True
                        print(f'  {GRN}●{NC}  Lucid          {GRN}connected{NC}')
                    elif state_key == 'lucid_waiting' and not self._lucid_ok:
                        print(f'  {YEL}○{NC}  Lucid          {YEL}waiting...{NC}')
                return
        if msg.level >= 40:
            key = f'{msg.name}:{msg.msg[:60]}'
            if not self._node_states.get(key):
                self._node_states[key] = True
                level = {40: 'ERROR', 50: 'FATAL'}.get(msg.level, 'ERROR')
                self._log(f'{level:<8} [{msg.name}] {msg.msg}')
                self._error_count += 1

    # ── MaxVis ────────────────────────────────────────────────────────────────
    def _poll_maxvis(self):
        threading.Thread(target=self._check_maxvis_signal, daemon=True).start()

    def _check_maxvis_signal(self):
        signal = maxvis_has_signal(self._v4l2_device)
        if signal and not self._maxvis_active:
            self._maxvis_active = True
            self._maxvis_on_since = time.time()
            self._log('MAXVIS   ON')
            print(f'  {GRN}●{NC}  MaxVis         {GRN}ON{NC}')
        elif not signal and self._maxvis_active:
            self._maxvis_active = False
            if self._maxvis_on_since is not None:
                self._maxvis_on_time += time.time() - self._maxvis_on_since
                self._maxvis_on_since = None
            self._log('MAXVIS   OFF')
            print(f'  {YEL}○{NC}  MaxVis         {YEL}OFF{NC}')

    # ── Trigger ───────────────────────────────────────────────────────────────
    def _trigger_cb(self, msg: Empty):
        self._trigger_count += 1
        mv = 'ON' if self._maxvis_active else 'OFF'
        self._log(f'TRIGGER  #{self._trigger_count} — MaxVis:{mv}')
        print(f'  {GRN}●{NC}  trigger #{self._trigger_count:<6} MaxVis:{mv}')

    # ── Email ─────────────────────────────────────────────────────────────────
    def _startup_email_body(self) -> str:
        ts = self._startup_time.strftime('%Y-%m-%d %H:%M:%S')
        return (
            f'TankerVision unit is online.\n\n'
            f'Unit:       {self._unit_name}\n'
            f'Plane:      {self._plane}\n'
            f'Province:   {self._province}\n'
            f'Time:       {ts}\n'
            f'Mode:       {self._mode}\n'
            f'Session:    {self._session_dir}\n\n'
            f'Chrony:     {"synced"      if self._chrony_ok  else "NOT SYNCED"}\n'
            f'PTP master: {"active"      if self._ptp_master else "not active"}\n'
            f'PTP slave:  {"locked"      if self._ptp_slave  else "uncalibrated"}\n'
            f'GNSS:       {"recording"   if self._gnss_ok    else "not running"}\n'
            f'IMU:        {"running"     if self._imu_ok     else "not running"}\n'
        )

    def _try_send_startup_email(self):
        if not self._email_to:
            return
        subject = f'TankerVision Online — {self._unit_name} ({self._plane})'
        ok = send_email_msmtp(
            self._email_to, subject, self._startup_email_body(), self._msmtprc)
        if ok:
            self._email_sent = True
            self._log('STATUS   Startup email sent')
            print(f'  {GRN}●{NC}  email          {GRN}sent{NC}\n')
        else:
            self._log('STATUS   Startup email failed — retry in 5 min')

    def _retry_email(self):
        if not self._email_sent:
            threading.Thread(target=self._try_send_startup_email, daemon=True).start()

    # ── Shutdown ──────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._write_footer()
        self._log_file.flush()
        self._log_file.close()
        self._copy_ros_log()
        super().destroy_node()

    def _copy_ros_log(self):
        ros_log = os.path.expanduser('~/.ros/log/latest')
        if not os.path.exists(ros_log):
            return
        dest = os.path.join(self._session_dir, 'ros_log')
        try:
            shutil.copytree(ros_log, dest)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()