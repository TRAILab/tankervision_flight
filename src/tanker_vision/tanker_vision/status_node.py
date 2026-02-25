import io
import sys
import os
import math
import socket
import shutil
import serial
import subprocess
import threading
import logging

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.node import Node
from .send_email import send_email, EmailError  # adjust import as needed

# ─── Logging setup ─────────────────────────────────────────────────────────────
_log_stream = io.StringIO()
_stream_handler = logging.StreamHandler(_log_stream)
_stream_handler.setLevel(logging.INFO)
_formatter = logging.Formatter(
    '[%(asctime)s] %(levelname)s %(name)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
_stream_handler.setFormatter(_formatter)

# ─── Configure the console handler ─────────────────────────────
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.INFO)
console_handler.setFormatter(_formatter)

# ─── Grab your logger and attach only the console & in-memory handlers ──────
_py_logger = logging.getLogger('status_node')
_py_logger.setLevel(logging.CRITICAL)
_py_logger.handlers.clear()
_py_logger.addHandler(console_handler)
_py_logger.addHandler(_stream_handler)
_py_logger.propagate = False

def chrony_has_pps():
    try:
        result = subprocess.run(
            ["chronyc", "sources"],
            capture_output=True,
            text=True,
            check=True
        )
    except subprocess.CalledProcessError:
        return False

    if "506" in result.stdout:
        return False

    parsing = False
    for line in result.stdout.splitlines():
        if line.strip().startswith("===="):
            parsing = True
            continue
        if not parsing:
            continue
        if "PPS" in line:
            cols = line.split(None, 8)
            if len(cols) >= 5:
                try:
                    reach = int(cols[4], 8)
                    return (reach & 0x01) != 0
                except ValueError:
                    pass
    return False

def has_internet():
    try:
        socket.create_connection(("www.google.com", 80), timeout=1.0)
        return True
    except OSError:
        return False

def get_free_space_percentage(path="/"):
    usage = shutil.disk_usage(path)
    if usage.total == 0:
        return "00"
    percent = int((usage.free / usage.total) * 100)
    return f"{min(percent, 99):02d}"

def is_disk_mounted(mount_point="/mnt/wildfire"):
    return os.path.ismount(mount_point)

class StateHandler:
    def __init__(self, name, ok_state, no_heartbeat_state):
        self.name = name
        self.state = None
        self.last_heartbeat = None
        self.ok_state = ok_state
        self.no_heartbeat_state = no_heartbeat_state

    def update_heartbeat(self, now, new_state=None):
        self.last_heartbeat = now
        self._set_state(new_state or self.ok_state)

    def check_heartbeat_timeout(self, now, timeout_sec):
        if (self.last_heartbeat is None or
            (now - self.last_heartbeat).nanoseconds / 1e9 > timeout_sec):
            self._set_state(self.no_heartbeat_state)

    def _set_state(self, new_state):
        if self.state != new_state:
            self.state = new_state
            _py_logger.critical(f"{self.name} state changed: {new_state}")

class StatusNode(Node):
    def __init__(self):
        super().__init__('status_node')

        # Thread pool for all blocking tasks
        self._executor = ThreadPoolExecutor(max_workers=4)

        self.is_in_air = False
        self.send_landing_email = False
        self.send_startup_email = True
        self.camera_time_sync_state = None
        self.internet_state = None
        self.time_sync_state = None
        self.velocity_msg_count = 0

        # ─── State handlers ─────────────────────────────
        self.gps       = StateHandler("GPS",    "GPS_OK",    "GPS_NO_HEARTBEAT")
        self.imu       = StateHandler("IMU",    "IMU_OK",    "IMU_NO_HEARTBEAT")
        self.camera    = StateHandler("CAMERA","CAMERA_RECEIVING", "CAMERA_NO_HEARTBEAT")
        self.recording = StateHandler("RECORDING", "RECORDING_RECORDING", "RECORDING_NO_HEARTBEAT")

        # ─── Subscriptions ────────────────────────────────
        self.create_subscription(String, 'gps_heartbeat',       self.gps_heartbeat_callback,       10)
        self.create_subscription(String, 'camera_heartbeat',    self.camera_heartbeat_callback,    10)
        self.create_subscription(String, 'imu/heartbeat',       self.imu_heartbeat_callback,       10)
        self.create_subscription(String, 'record_data/status',  self.recording_heartbeat_callback, 10)
        self.create_subscription(
            Vector3Stamped, '/filter/velocity', self.velocity_callback, 10
        )

        # ─── Timers ────────────────────────────────────────
        self.create_timer(1.25, self.check_gps_heartbeat)
        self.create_timer(1.25, self.check_camera_heartbeat)
        self.create_timer(1.0,  self.check_imu_heartbeat)
        self.create_timer(1.25, self.check_recording_heartbeat)
        self.create_timer(5.0,  self.check_internet_connection)
        self.create_timer(10.0, self.check_time_sync)

        # ─── Initial storage check ────────────────────────
        if is_disk_mounted():
            try:
                _ = float(get_free_space_percentage("/mnt/wildfire"))
            except Exception as e:
                _py_logger.critical(f"Storage check error: {e}")
        else:
            _py_logger.critical("Storage check skipped: Disk not mounted.")

    # ─── Heartbeat callbacks & checks ─────────────────────
    def gps_heartbeat_callback(self, msg):
        self.gps.update_heartbeat(self.get_clock().now())

    def camera_heartbeat_callback(self, msg):
        now = self.get_clock().now()
        data = msg.data.lower()
        state = "CAMERA_RECEIVING"
        if "timeout" in data:
            state = "CAMERA_TIMEOUT"
        elif "error" in data or "exception" in data:
            state = "CAMERA_ERROR"
        self.camera.update_heartbeat(now, state)

        sync_state = "CAMERA_TIME_SYNCED" if "slave" in data else "CAMERA_TIME_NOT_SYNCED"
        if sync_state != self.camera_time_sync_state:
            self.camera_time_sync_state = sync_state
            _py_logger.critical(f"Camera Time Sync state changed: {sync_state}")

    def imu_heartbeat_callback(self, msg):
        self.imu.update_heartbeat(self.get_clock().now())

    def recording_heartbeat_callback(self, msg):
        now = self.get_clock().now()
        low = msg.data.lower()
        if "recording" in low:
            st = "RECORDING_RECORDING"
        elif "scanning" in low:
            st = "RECORDING_SCANNING"
        else:
            return
        self.recording.update_heartbeat(now, st)

    def check_gps_heartbeat(self):
        self.gps.check_heartbeat_timeout(self.get_clock().now(), 2.0)

    def check_camera_heartbeat(self):
        self.camera.check_heartbeat_timeout(self.get_clock().now(), 3.0)

    def check_imu_heartbeat(self):
        self.imu.check_heartbeat_timeout(self.get_clock().now(), 2.0)

    def check_recording_heartbeat(self):
        self.recording.check_heartbeat_timeout(self.get_clock().now(), 2.0)

    # ─── Internet & Time Sync Checks ─────────────────────
    def check_internet_connection(self):
        threading.Thread(target=self._check_internet_bg, daemon=True).start()

    def _check_internet_bg(self):
        connected = has_internet()
        state = "INTERNET_OK" if connected else "INTERNET_NO_CONNECTION"
        if state != self.internet_state:
            self.internet_state = state
            _py_logger.critical(f"Internet state changed: {state}")

            if connected and self.send_landing_email:
                self.send_landing_email = False
                self._executor.submit(self._send_status_email,
                                      'Landing Alert',
                                      'The plane has landed. System status:')
                self._executor.submit(self.send_sms,
                                      'LANDING: The plane has landed. Another successful flight!')
            elif connected and self.send_startup_email:
                self.send_startup_email = False
                self._executor.submit(send_email,
                                      'Startup Alert',
                                      'The plane has started up.')
                self._executor.submit(self.send_sms,
                                      'STARTUP: The plane has started up. Let\'s go!')

    def check_time_sync(self):
        threading.Thread(target=self._check_time_sync_bg, daemon=True).start()

    def _check_time_sync_bg(self):
        pps_ok = chrony_has_pps()
        state = "TIME_SYNC_PPS" if pps_ok else "TIME_SYNC_NO_PPS"
        if state != self.time_sync_state:
            self.time_sync_state = state
            _py_logger.critical(f"Time Sync state changed: {state}")

    # ─── Recording Control & Velocity ─────────────────────
    def control_cellular(self, enable: bool):
        try:
            ser = serial.Serial('/dev/ttyUSB3', 115200, timeout=1)
            if enable:
                ser.write(b'AT+CFUN=1\r\n')
                self.send_landing_email = True
            else:
                ser.write(b'AT+CFUN=4\r\n')
            ser.read(128)
        except Exception as e:
            _py_logger.critical(f"Failed to control cellular: {e}")
        finally:
            ser.close()

    def send_sms(self, message: str,
                 phone_number: str = "+19022099739",
                 port: str = '/dev/ttyUSB3',
                 baud: int = 115200):
        try:
            ser = serial.Serial(port, baud, timeout=0.5)
            ser.write(b'AT+CMGF=1\r')
            ser.readline()
            ser.write(f'AT+CMGS="{phone_number}"\r'.encode())
            if b'>' not in ser.read_until(b'>'):
                _py_logger.critical("SMS failed: no prompt")
                return
            ser.write(message.encode() + b'\x1A')
            if b'OK' not in ser.read_until(b'OK'):
                _py_logger.critical("SMS failed: no OK")
        except Exception as e:
            _py_logger.critical(f"SMS failed: {e}")
        finally:
            try:
                ser.close()
            except:
                pass

    def _takeoff_procedure(self, mag):
        # 1) Send SMS synchronously
        sms_msg = f"TAKEOFF: The plane is taking off. Velocity={mag:.2f} m/s. It's go time!"
        try:
            self.send_sms(sms_msg)
        except Exception as e:
            _py_logger.critical(f"Takeoff SMS failed: {e}")

        # 2) Send the status email synchronously
        #    (capture & clear logs, run du, send_email)
        logs = _log_stream.getvalue()
        _log_stream.truncate(0)
        _log_stream.seek(0)
        try:
            du_output = subprocess.check_output(
                ['du', '-h', '/mnt/wildfire/data/'],
                text=True, stderr=subprocess.STDOUT
            )
        except subprocess.CalledProcessError as e:
            du_output = f"du error: {e.output or e}"
        full_body = (
            "Takeoff Alert: The plane is taking off.\n\n"
            f"=== NODE LOGS ===\n{logs}\n\n"
            f"=== DATA USAGE ===\n{du_output}"
        )
        try:
            send_email(subject='Takeoff Alert', body=full_body)
            _py_logger.critical("Status email sent: Takeoff Alert")
        except EmailError as e:
            _py_logger.critical(f"Takeoff email failed: {e}")

        # 3) Finally, turn off the cellular modem
        self.control_cellular(False)

    def velocity_callback(self, msg):
        self.velocity_msg_count += 1
        if self.velocity_msg_count % 100:
            return
        mag = math.sqrt(msg.vector.x**2 +
                        msg.vector.y**2 +
                        msg.vector.z**2)

        if mag > 14.0 and not self.is_in_air:
            self.is_in_air = True
            _py_logger.critical(f"Takeoff detected: {mag:.2f} m/s")
            self._executor.submit(self._takeoff_procedure, mag)
        elif mag < 1.0 and self.is_in_air:
            self.is_in_air = False
            _py_logger.critical(f"Landing detected: {mag:.2f} m/s")
            self._executor.submit(self.control_cellular, True)

    # ─── Email Status Reports ──────────────────────────────
    def _send_status_email(self, subject: str, body_intro: str):
        # capture & clear logs immediately
        logs = _log_stream.getvalue()
        _log_stream.truncate(0)
        _log_stream.seek(0)

        # offload the heavy work
        self._executor.submit(self._du_and_email, subject, body_intro, logs)

    def _du_and_email(self, subject: str, body_intro: str, logs: str):
        try:
            du_output = subprocess.check_output(
                ['du', '-h', '/mnt/wildfire/data/'],
                text=True,
                stderr=subprocess.STDOUT
            )
        except subprocess.CalledProcessError as e:
            du_output = f"du error: {e.output or e}"

        full_body = (
            f"{body_intro}\n\n"
            f"=== NODE LOGS ===\n{logs}\n\n"
            f"=== DATA USAGE ===\n{du_output}"
        )

        try:
            send_email(subject=subject, body=full_body)
            _py_logger.critical(f"Status email sent: {subject}")
        except EmailError as e:
            _py_logger.critical(f"Error sending email: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
