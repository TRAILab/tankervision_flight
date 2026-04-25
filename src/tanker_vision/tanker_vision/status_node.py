#!/usr/bin/env python3
"""
status_node.py — TankerVision flight status monitor.

Responsibilities:
  - On startup: send email notification (retry every 5 min until success)
  - Write flight log to /mnt/storage/flight_log_<timestamp>.txt
  - Subscribe to /rosout — log WARN/ERROR from all nodes
  - Monitor /cam1/image_raw rate — detect MaxVis on/off
  - Subscribe to /save_images_trigger — log each trigger + MaxVis state
  - On shutdown: write session summary to log
"""

import os
import subprocess
import threading
import time
import yaml
from datetime import datetime
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image
from std_msgs.msg import Empty


# ── QoS ──────────────────────────────────────────────────────────────────────
_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
_ROSOUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
)


# ── YAML loader ───────────────────────────────────────────────────────────────
def _load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


# ── msmtp email ───────────────────────────────────────────────────────────────
def send_email_msmtp(to: str, subject: str, body: str, msmtprc: str = None) -> bool:
    """
    Send email via msmtp. Returns True on success, False on failure.
    """
    try:
        cmd = ['msmtp']
        if msmtprc and os.path.exists(msmtprc):
            cmd += ['-C', msmtprc]
        cmd.append(to)
        msg = f"To: {to}\nSubject: {subject}\n\n{body}"
        result = subprocess.run(
            cmd,
            input=msg,
            capture_output=True,
            text=True,
            timeout=15,
        )
        return result.returncode == 0
    except Exception:
        return False


# ── Status node ───────────────────────────────────────────────────────────────
class StatusNode(Node):

    MAXVIS_TIMEOUT_SEC = 3.0     # seconds of silence before MaxVis considered off
    EMAIL_RETRY_SEC    = 300.0   # retry email every 5 minutes

    def __init__(self):
        super().__init__('status_node')

        # ── Load config ───────────────────────────────────────────
        yaml_path = self.declare_parameter(
            'config', '/home/flight/tankervision_flight/config/tankervision.yaml'
        ).value
        try:
            cfg = _load_yaml(yaml_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            cfg = {}

        unit_cfg  = cfg.get('unit', {})
        notif_cfg = cfg.get('notifications', {})
        stor_cfg  = cfg.get('storage', {})

        self._unit_name   = unit_cfg.get('name', 'unknown')
        self._plane       = unit_cfg.get('plane_number', 'unknown')
        self._province    = unit_cfg.get('province', 'unknown')
        self._mode        = cfg.get('mode', 'testing')
        self._email_to    = notif_cfg.get('email_to', '')
        self._storage     = stor_cfg.get('root', '/mnt/storage')
        self._msmtprc     = notif_cfg.get('msmtprc', '/home/argus/.msmtprc')

        # ── State ─────────────────────────────────────────────────
        self._startup_time     = datetime.now()
        self._email_sent       = False
        self._maxvis_active    = False
        self._last_maxvis_msg  = None   # rclpy time
        self._trigger_count    = 0
        self._error_count      = 0
        self._maxvis_on_time   = 0.0    # accumulated seconds
        self._maxvis_on_since  = None   # wall time when MaxVis last came on
        self._lock             = threading.Lock()

        # ── Open log file ─────────────────────────────────────────
        ts = self._startup_time.strftime('%Y%m%d_%H%M%S')
        log_path = os.path.join(self._storage, f'flight_log_{ts}.txt')
        os.makedirs(self._storage, exist_ok=True)
        self._log_file = open(log_path, 'w', buffering=1)  # line buffered
        self._write_header()

        # ── Subscriptions ─────────────────────────────────────────
        self.create_subscription(
            Log, '/rosout', self._rosout_cb, _ROSOUT_QOS)

        self.create_subscription(
            Image, '/cam1/image_raw', self._maxvis_cb, _BEST_EFFORT_QOS)

        self.create_subscription(
            Empty, '/save_images_trigger', self._trigger_cb, 10)

        # ── Timers ────────────────────────────────────────────────
        # Check MaxVis timeout every second
        self.create_timer(1.0, self._check_maxvis_timeout)

        # Email retry every 5 minutes
        self.create_timer(self.EMAIL_RETRY_SEC, self._retry_email)

        # Try startup email immediately in background
        threading.Thread(target=self._try_send_startup_email, daemon=True).start()

        self._log(f'STATUS   status_node online — mode: {self._mode}')
        self.get_logger().info('StatusNode started')

    # ── Log helpers ───────────────────────────────────────────────────────────
    def _ts(self) -> str:
        return datetime.now().strftime('%H:%M:%S')

    def _log(self, msg: str):
        line = f'[{self._ts()}] {msg}\n'
        with self._lock:
            self._log_file.write(line)

    def _write_header(self):
        ts = self._startup_time.strftime('%Y-%m-%d %H:%M:%S %Z')
        lines = [
            '=' * 80,
            'TANKERVISION FLIGHT LOG',
            '=' * 80,
            f'Unit:       {self._unit_name}',
            f'Plane:      {self._plane}',
            f'Province:   {self._province}',
            f'Start time: {ts}',
            f'Mode:       {self._mode}',
            '=' * 80,
            '',
        ]
        with self._lock:
            self._log_file.write('\n'.join(lines) + '\n')

    def _write_footer(self):
        end_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S %Z')
        duration = datetime.now() - self._startup_time
        h, rem = divmod(int(duration.total_seconds()), 3600)
        m, s   = divmod(rem, 60)

        # Finalize MaxVis time
        if self._maxvis_on_since is not None:
            self._maxvis_on_time += time.time() - self._maxvis_on_since

        mv_h, mv_rem = divmod(int(self._maxvis_on_time), 3600)
        mv_m, mv_s   = divmod(mv_rem, 60)

        lines = [
            '',
            '=' * 80,
            f'Session end:    {end_time}',
            f'Duration:       {h:02d}:{m:02d}:{s:02d}',
            f'Triggers:       {self._trigger_count}',
            f'MaxVis on time: {mv_h:02d}:{mv_m:02d}:{mv_s:02d}',
            f'Errors logged:  {self._error_count}',
            '=' * 80,
        ]
        with self._lock:
            self._log_file.write('\n'.join(lines) + '\n')

    # ── /rosout callback ──────────────────────────────────────────────────────
    def _rosout_cb(self, msg: Log):
        # Only log WARN (30) and above; skip our own node
        if msg.level < 30 or msg.name == 'status_node':
            return
        level = {30: 'WARN', 40: 'ERROR', 50: 'FATAL'}.get(msg.level, 'LOG')
        self._log(f'{level:<8} [{msg.name}] {msg.msg}')
        if msg.level >= 40:
            self._error_count += 1

    # ── /cam1/image_raw callback ───────────────────────────────────────────────
    def _maxvis_cb(self, msg: Image):
        self._last_maxvis_msg = self.get_clock().now()
        if not self._maxvis_active:
            self._maxvis_active = True
            self._maxvis_on_since = time.time()
            self._log('MAXVIS   Analog stream ON')

    def _check_maxvis_timeout(self):
        if not self._maxvis_active:
            return
        if self._last_maxvis_msg is None:
            return
        elapsed = (self.get_clock().now() - self._last_maxvis_msg).nanoseconds / 1e9
        if elapsed > self.MAXVIS_TIMEOUT_SEC:
            self._maxvis_active = False
            if self._maxvis_on_since is not None:
                self._maxvis_on_time += time.time() - self._maxvis_on_since
                self._maxvis_on_since = None
            self._log('MAXVIS   Analog stream OFF')

    # ── /save_images_trigger callback ─────────────────────────────────────────
    def _trigger_cb(self, msg: Empty):
        self._trigger_count += 1
        mv_state = 'ON' if self._maxvis_active else 'OFF'
        self._log(f'TRIGGER  Save trigger #{self._trigger_count} — MaxVis: {mv_state}')

    # ── Email ─────────────────────────────────────────────────────────────────
    def _startup_email_body(self) -> str:
        ts = self._startup_time.strftime('%Y-%m-%d %H:%M:%S')
        return (
            f'TankerVision unit is online.\n\n'
            f'Unit:     {self._unit_name}\n'
            f'Plane:    {self._plane}\n'
            f'Province: {self._province}\n'
            f'Time:     {ts}\n'
            f'Mode:     {self._mode}\n'
        )

    def _try_send_startup_email(self):
        if not self._email_to:
            return
        subject = f'TankerVision Online — {self._unit_name} ({self._plane})'
        success = send_email_msmtp(self._email_to, subject, self._startup_email_body(), self._msmtprc)
        if success:
            self._email_sent = True
            self._log('STATUS   Startup email sent')
        else:
            self._log('STATUS   Startup email failed — will retry')

    def _retry_email(self):
        if self._email_sent:
            return
        threading.Thread(target=self._try_send_startup_email, daemon=True).start()

    # ── Shutdown ──────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._write_footer()
        self._log_file.flush()
        self._log_file.close()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────
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