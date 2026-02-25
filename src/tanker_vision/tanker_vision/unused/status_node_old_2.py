#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading
import subprocess
from .LCD_drive import StatusDisplay, get_free_space_percentage
import os
import math
from geometry_msgs.msg import Vector3Stamped
import serial

def chrony_has_pps():
    try:
        result = subprocess.run([
            "chronyc", "sources"
        ], capture_output=True, text=True, check=True)
    except subprocess.CalledProcessError:
        return False

    if "506" in result.stdout:
        return False

    lines = result.stdout.splitlines()
    parsing_data = False
    for line in lines:
        if line.strip().startswith("===="):
            parsing_data = True
            continue
        if not parsing_data:
            continue
        if "PPS" in line:
            columns = line.split(None, 8)
            if len(columns) >= 5:
                try:
                    reach_val = int(columns[4], 8)
                    return (reach_val & 0x01) != 0
                except ValueError:
                    pass
    return False

def has_internet():
    try:
        socket.create_connection(("www.google.com", 80), timeout=1.0)
        return True
    except OSError:
        return False

def is_disk_mounted(mount_point="/mnt/wildfire"):
    return os.path.ismount(mount_point)

class StateHandler:
    def __init__(self, name, logger_fn, ok_value, no_heartbeat_value=None):
        self.name = name
        # self.display_fn = display_fn
        self.logger_fn = logger_fn
        self.state = None
        self.last_heartbeat = None
        self.ok_value = ok_value
        self.no_heartbeat_value = no_heartbeat_value

    def update_heartbeat(self, now, new_state=None):
        self.last_heartbeat = now
        self.set_state(new_state or self.ok_value)

    def check_heartbeat_timeout(self, now, timeout_sec):
        if self.last_heartbeat is None or (now - self.last_heartbeat).nanoseconds / 1e9 > timeout_sec:
            self.set_state(self.no_heartbeat_value)

    def set_state(self, new_state):
        if self.state != new_state:
            self.state = new_state
            # self.display_fn(new_state)
            self.logger_fn(f"{self.name} state changed: {new_state}")

class StatusNode(Node):
    def __init__(self):
        super().__init__('status_node')
        #self.display = StatusDisplay()
        self.is_in_air = False

        self.gps = StateHandler("GPS", self.get_logger().info, "GPS_OK", "GPS_NO_HEARTBEAT")
        self.imu = StateHandler("IMU", self.get_logger().info, "IMU_OK", "IMU_NO_HEARTBEAT")
        self.camera = StateHandler("CAMERA", self.get_logger().info, "CAMERA_RECEIVING", "CAMERA_NO_HEARTBEAT")
        self.recording = StateHandler("RECORDING", self.get_logger().info, "RECORDING_RECORDING", "RECORDING_NO_HEARTBEAT")

        self.camera_time_sync_state = None
        self.internet_state = None
        self.time_sync_state = None

        self.create_subscription(String, 'gps_heartbeat', self.gps_heartbeat_callback, 10)
        self.create_subscription(String, 'camera_heartbeat', self.camera_heartbeat_callback, 10)
        self.create_subscription(String, 'imu/heartbeat', self.imu_heartbeat_callback, 10)
        self.create_subscription(String, 'record_data/status', self.recording_heartbeat_callback, 10)
        self.create_subscription(Vector3Stamped, '/filter/velocity', self.velocity_callback, 10)

        self.create_timer(1.0, self.check_gps_heartbeat)
        self.create_timer(1.0, self.check_camera_heartbeat)
        self.create_timer(1.0, self.check_imu_heartbeat)
        self.create_timer(1.0, self.check_recording_heartbeat)
        self.create_timer(5.0, self.check_internet_connection)
        self.create_timer(10.0, self.check_time_sync)

        self.velocity_msg_count = 0

        if is_disk_mounted():
            self.check_storage()
        else:
            self.get_logger().warn("Storage check skipped: Disk not mounted.")

    def check_storage(self):
        percent = get_free_space_percentage("/mnt/wildfire")
        #self.display.set_storage_value(percent)

    def control_cellular(self, enable: bool):
        """
        Controls the cellular modem by sending AT commands to turn it on or off.
        
        :param enable: True to turn on cellular, False to turn it off.
        """
        try:
            # Open the serial port with appropriate settings:
            ser = serial.Serial('/dev/ttyUSB3', 115200, timeout=1)
            if enable:
                ser.write(b'AT+CFUN=1\r\n')  # Set to full functionality mode
                self.get_logger().info("Turning on cellular modem.")
            else:
                ser.write(b'AT+CFUN=4\r\n')  # Set to airplane mode
                self.get_logger().info("Turning off cellular modem.")
            response = ser.read(128)
            print("Response:", response.decode(errors='ignore'))
        except Exception as e:
            self.get_logger().error(f"Failed to control cellular: {e}")
        finally:
            ser.close()

    def velocity_callback(self, msg):
        self.velocity_msg_count += 1
        if self.velocity_msg_count % 100 != 0:
            return

        mag = math.sqrt(msg.vector.x**2 + msg.vector.y**2 + msg.vector.z**2)
        if mag > 14.0 and not self.is_in_air:
            #self.display.set_system_state(True)
            self.is_in_air = True
            self.get_logger().warn(f"Takeoff detected: {mag:.2f} m/s")
            self.control_cellular(False)  # Turn off cellular modem
        elif mag < 1.0 and self.is_in_air:
            #self.display.set_system_state(False)
            self.is_in_air = False
            self.get_logger().info(f"Landing detected: {mag:.2f} m/s")
            self.control_cellular(True)  # Turn on cellular modem

    def gps_heartbeat_callback(self, msg):
        self.gps.update_heartbeat(self.get_clock().now())

    def camera_heartbeat_callback(self, msg):
        now = self.get_clock().now()
        state = "CAMERA_RECEIVING"
        if "timeout" in msg.data.lower():
            state = "CAMERA_TIMEOUT"
        elif "error" in msg.data.lower() or "exception" in msg.data.lower():
            state = "CAMERA_ERROR"
        self.camera.update_heartbeat(now, state)

        new_sync_state = "CAMERA_TIME_SYNCED" if "slave" in msg.data.lower() else "CAMERA_TIME_NOT_SYNCED"
        if self.camera_time_sync_state != new_sync_state:
            self.camera_time_sync_state = new_sync_state
            self.check_full_synchronization()
            self.get_logger().info(f"Camera Time Sync state changed: {new_sync_state}")

    def imu_heartbeat_callback(self, msg):
        self.imu.update_heartbeat(self.get_clock().now())

    def recording_heartbeat_callback(self, msg):
        now = self.get_clock().now()
        if "recording" in msg.data.lower():
            state = "RECORDING_RECORDING"
        elif "scanning" in msg.data.lower():
            state = "RECORDING_SCANNING"
        else:
            state = None
        if state:
            self.recording.update_heartbeat(now, state)

    def check_gps_heartbeat(self):
        self.gps.check_heartbeat_timeout(self.get_clock().now(), 2.0)

    def check_camera_heartbeat(self):
        self.camera.check_heartbeat_timeout(self.get_clock().now(), 3.0)

    def check_imu_heartbeat(self):
        #self.display.heartbeat()
        self.imu.check_heartbeat_timeout(self.get_clock().now(), 2.0)

    def check_recording_heartbeat(self):
        self.recording.check_heartbeat_timeout(self.get_clock().now(), 2.0)

    def check_internet_connection(self):
        threading.Thread(target=self._check_internet_bg, daemon=True).start()

    def _check_internet_bg(self):
        connected = has_internet()
        new_state = "INTERNET_OK" if connected else "INTERNET_NO_CONNECTION"
        if self.internet_state != new_state:
            self.internet_state = new_state
            #self.display.set_cellular_status(connected)
            log = self.get_logger().info if connected else self.get_logger().warn
            log(f"Internet state changed: {new_state}")

    def check_time_sync(self):
        threading.Thread(target=self._check_time_sync_bg, daemon=True).start()

    def _check_time_sync_bg(self):
        pps_ok = chrony_has_pps()
        new_state = "TIME_SYNC_PPS" if pps_ok else "TIME_SYNC_NO_PPS"
        if self.time_sync_state != new_state:
            self.time_sync_state = new_state
            self.check_full_synchronization()
            log = self.get_logger().info if pps_ok else self.get_logger().warn
            log(f"Time Sync state changed: {new_state}")

    def check_full_synchronization(self):
        synced = self.camera_time_sync_state == "CAMERA_TIME_SYNCED" and self.time_sync_state == "TIME_SYNC_PPS"
        #self.display.set_timesync_status(synced)

    def _camera_display(self, state):
        mapping = {
            "CAMERA_RECEIVING": "OKAY",
            "CAMERA_TIMEOUT": "TOUT",
            "CAMERA_ERROR": "ERR!",
            "CAMERA_NO_HEARTBEAT": "----"
        }
        #self.display.set_camera_state(mapping.get(state, "----"))

    def _recording_display(self, state):
        return
        # if state == "RECORDING_RECORDING":
        #     #self.display.set_recording_status(True)
        # elif state == "RECORDING_SCANNING":
        #     #self.display.set_recording_status(False)
        # elif state == "RECORDING_NO_HEARTBEAT":
        #     #self.display.set_recording_status(False, True)

def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()