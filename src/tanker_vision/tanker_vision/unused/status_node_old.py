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
    """
    Runs 'chronyc sources' and returns True if PPS is found with reach=377,
    otherwise False.
    """
    try:
        result = subprocess.run(
            ["chronyc", "sources"],
            capture_output=True,
            text=True,
            check=True
        )
    except subprocess.CalledProcessError as e:
        return False
    
    if "506" in result.stdout:
        # 506 cannot talk to daemon
        return False

    lines = result.stdout.splitlines()
    parsing_data = False
    
    for line in lines:
        line_stripped = line.strip()
        # The line of equal signs marks the start of source table
        if line_stripped.startswith("===="):
            parsing_data = True
            continue
        
        if not parsing_data:
            continue
        
        if "PPS" in line:
            columns = line.split(None, 8)
            if len(columns) >= 5:
                reach_str = columns[4]  # e.g. "377"
                try:
                    reach_val = int(reach_str, 8)  # parse as octal
                    # Check the least significant bit
                    return (reach_val & 0x01) != 0
                except ValueError:
                    pass
    return False


def has_internet() -> bool:
    """
    Attempt to connect to a known server to check for internet connectivity.
    """
    try:
        socket.create_connection(("www.google.com", 80), timeout=1.0)
        return True
    except OSError:
        return False

def is_disk_mounted(mount_point="/mnt/wildfire"):
    return os.path.ismount(mount_point)

if is_disk_mounted():
    print("Disk is mounted.")
else:
    print("Disk is not mounted.")

class StatusNode(Node):
    def __init__(self):
        super().__init__('status_node')
        self.display = StatusDisplay()

        self.is_in_air = False
        
        # Subscriptions for GPS, camera, IMU, and recording node heartbeats.
        self.gps_heartbeat_sub = self.create_subscription(
            String, 'gps_heartbeat', self.gps_heartbeat_callback, 10)
        self.camera_heartbeat_sub = self.create_subscription(
            String, 'camera_heartbeat', self.camera_heartbeat_callback, 10)
        self.imu_heartbeat_sub = self.create_subscription(
            String, 'imu/heartbeat', self.imu_heartbeat_callback, 10)
        self.recording_heartbeat_sub = self.create_subscription(
            String, 'record_data/status', self.recording_heartbeat_callback, 10)

        # Track GPS heartbeat
        self.last_gps_heartbeat_time = None
        self.gps_state = None  # "GPS_OK", "GPS_NO_HEARTBEAT"

        # Track Camera heartbeat
        self.last_camera_heartbeat_time = None
        self.camera_state = None  # "CAMERA_RECEIVING", "CAMERA_ERROR", etc.
        
        # NEW: Track camera time synchronization
        # "CAMERA_TIME_SYNCED" or "CAMERA_TIME_NOT_SYNCED"
        self.camera_time_sync_state = None

        # Track IMU heartbeat
        self.last_imu_heartbeat_time = None
        self.imu_state = None  # "IMU_OK", "IMU_NO_HEARTBEAT"

        # Track Recording heartbeat
        self.last_recording_heartbeat_time = None
        self.recording_state = None # "RECORDING_RECORDING", "RECORDING_SCANNING" "RECORDING_NO_HEARTBEAT"

        # Track internet state
        self.internet_state = None  # "INTERNET_OK", "INTERNET_NO_CONNECTION"

        # Track Time Sync/PPS state
        self.time_sync_state = None  # "TIME_SYNC_PPS" or "TIME_SYNC_NO_PPS"

        # Create timers to periodically check for missed heartbeats.
        self.gps_check_timer = self.create_timer(1.0, self.check_gps_heartbeat)
        self.camera_check_timer = self.create_timer(1.0, self.check_camera_heartbeat)
        self.imu_check_timer = self.create_timer(1.0, self.check_imu_heartbeat)
        self.recording_check_timer = self.create_timer(1.0, self.check_recording_heartbeat)

        # Create a timer to check internet connectivity every 5 seconds.
        self.internet_check_timer = self.create_timer(5.0, self.check_internet_connection)

        # Create a timer to check PPS/time sync every 10 seconds.
        self.time_sync_check_timer = self.create_timer(10.0, self.check_time_sync)

        # Counter to track how many velocity messages have been received.
        self.velocity_msg_count = 0

        # Subscribe to the velocity topic.
        self.velocity_sub = self.create_subscription(
            Vector3Stamped, '/filter/velocity', self.velocity_callback, 10)
        if is_disk_mounted():
            self.check_storage()
        else:
            self.get_logger().warn("Storage check skipped: Disk not mounted.")
        

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
    
    def velocity_callback(self, msg: Vector3Stamped):
        # Increment the counter each time a message is received.
        self.velocity_msg_count += 1

        # Only check every 100th message.
        if self.velocity_msg_count % 100 != 0:
            return

        # Process the message if it's the 100th one.
        vx = msg.vector.x
        vy = msg.vector.y
        vz = msg.vector.z
        magnitude = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

        if magnitude > 3.0 and not self.is_in_air:
            self.display.set_system_state(True)
            self.is_in_air = True
            self.get_logger().warn(f"Takeoff detected: {magnitude:.2f} m/s")
        elif magnitude < 0.1 and self.is_in_air:
            self.display.set_system_state(False)
            self.get_logger().info(f"Landing detected: {magnitude:.2f} m/s")

    def check_full_synchronization(self):
        if self.camera_time_sync_state == "CAMERA_TIME_SYNCED" and self.time_sync_state == "TIME_SYNC_PPS":
            self.display.set_timesync_status(True)
        else:
            self.display.set_timesync_status(False)
        return

    def check_storage(self):
        path_to_check = "/mnt/wildfire"  # e.g., root drive, or "/mnt/my_drive"
        free_percent = get_free_space_percentage(path_to_check)
        self.display.set_storage_value(free_percent)

    def gps_heartbeat_callback(self, msg: String):
        self.last_gps_heartbeat_time = self.get_clock().now()
        if self.gps_state != "GPS_OK":
            self.display.set_gps_status(True)
            self.gps_state = "GPS_OK"
            self.get_logger().info("GPS state changed: GPS_OK")

    def camera_heartbeat_callback(self, msg: String):
        """
        Camera heartbeat callback updates two things:
        1) The overall camera_state (CAMERA_RECEIVING, TIMEOUT, ERROR, etc.)
        2) The camera_time_sync_state based on whether the message contains "slave".
        """
        self.last_camera_heartbeat_time = self.get_clock().now()

        # --- 1) Overall camera state ---
        if "timeout" in msg.data.lower():
            new_state = "CAMERA_TIMEOUT"
            self.display.set_camera_state("TOUT")
        elif "error" in msg.data.lower() or "exception" in msg.data.lower():
            self.display.set_camera_state("ERR!")
            new_state = "CAMERA_ERROR"
        else:
            self.display.set_camera_state("OKAY")
            new_state = "CAMERA_RECEIVING"

        if self.camera_state != new_state:
            self.camera_state = new_state
            self.get_logger().info(f"Camera state changed: {self.camera_state}")

        # --- 2) Camera time sync status ---
        # Check if the message indicates the camera is in "slave" mode
        # (i.e. time-synchronized).
        if "slave" in msg.data.lower():
            new_time_sync_state = "CAMERA_TIME_SYNCED"
        else:
            new_time_sync_state = "CAMERA_TIME_NOT_SYNCED"

        if self.camera_time_sync_state != new_time_sync_state:
            self.camera_time_sync_state = new_time_sync_state
            self.check_full_synchronization()
            self.get_logger().info(f"Camera Time Sync state changed: {self.camera_time_sync_state}")

    def imu_heartbeat_callback(self, msg: String):
        self.last_imu_heartbeat_time = self.get_clock().now()
        if self.imu_state != "IMU_OK":
            self.display.set_imu_status(True)
            self.imu_state = "IMU_OK"
            self.get_logger().info("IMU state changed: IMU_OK")

    def recording_heartbeat_callback(self, msg: String):
        self.last_recording_heartbeat_time = self.get_clock().now()
        if "recording" in msg.data.lower():
            new_state = "RECORDING_RECORDING"
            self.display.set_recording_status(True)
        elif "scanning" in msg.data.lower():
            new_state = "RECORDING_SCANNING"
            self.display.set_recording_status(False)
        if self.recording_state != new_state:
            self.recording_state = new_state
            self.get_logger().info(f"Recording state changed: {self.recording_state}")

    def check_gps_heartbeat(self):
        now = self.get_clock().now()
        if self.last_gps_heartbeat_time is None:
            if self.gps_state != "GPS_NO_HEARTBEAT":
                self.gps_state = "GPS_NO_HEARTBEAT"
                self.get_logger().warn("GPS state changed: GPS_NO_HEARTBEAT (no heartbeat yet)")
            return
        elapsed = (now - self.last_gps_heartbeat_time).nanoseconds / 1e9
        if elapsed > 2.0:
            if self.gps_state != "GPS_NO_HEARTBEAT":
                self.display.set_gps_status(False)
                self.gps_state = "GPS_NO_HEARTBEAT"
                self.get_logger().warn(
                    f"GPS state changed: GPS_NO_HEARTBEAT (last heartbeat {elapsed:.2f}s ago)"
                )

    def check_camera_heartbeat(self):
        now = self.get_clock().now()
        if self.last_camera_heartbeat_time is None:
            if self.camera_state != "CAMERA_NO_HEARTBEAT":
                self.camera_state = "CAMERA_NO_HEARTBEAT"
                self.get_logger().warn("Camera state changed: CAMERA_NO_HEARTBEAT (no heartbeat yet)")
            return
        elapsed = (now - self.last_camera_heartbeat_time).nanoseconds / 1e9
        if elapsed > 2.0:
            if self.camera_state != "CAMERA_NO_HEARTBEAT":
                self.camera_state = "CAMERA_NO_HEARTBEAT"
                self.display.set_camera_state("----")
                self.get_logger().warn(
                    f"Camera state changed: CAMERA_NO_HEARTBEAT (last heartbeat {elapsed:.2f}s ago)"
                )

    def check_recording_heartbeat(self):
        now = self.get_clock().now()
        if self.last_recording_heartbeat_time is None:
            if self.recording_state != "RECORDING_NO_HEARTBEAT":
                self.recording_state = "RECORDING_NO_HEARTBEAT"
                self.get_logger().warn("Recording state changed: RECORDING_NO_HEARTBEAT (no heartbeat yet)")
            return
        elapsed = (now - self.last_recording_heartbeat_time).nanoseconds / 1e9
        if elapsed > 2.0:
            if self.recording_state != "RECORDING_NO_HEARTBEAT":
                self.display.set_recording_status(False, True)
                self.recording_state = "RECORDING_NO_HEARTBEAT"
                self.get_logger().warn(
                    f"Recording state changed: RECORDING_NO_HEARTBEAT (last heartbeat {elapsed:.2f}s ago)"
                )

    def check_imu_heartbeat(self):
        self.display.heartbeat()
        now = self.get_clock().now()
        if self.last_imu_heartbeat_time is None:
            if self.imu_state != "IMU_NO_HEARTBEAT":
                self.imu_state = "IMU_NO_HEARTBEAT"
                self.get_logger().warn("IMU state changed: IMU_NO_HEARTBEAT (no heartbeat yet)")
            return
        elapsed = (now - self.last_imu_heartbeat_time).nanoseconds / 1e9
        if elapsed > 2.0:
            if self.imu_state != "IMU_NO_HEARTBEAT":
                self.display.set_imu_status(False)
                self.imu_state = "IMU_NO_HEARTBEAT"
                self.get_logger().warn(
                    f"IMU state changed: IMU_NO_HEARTBEAT (last heartbeat {elapsed:.2f}s ago)"
                )

    def check_internet_connection(self):
        """
        Fire up a separate thread to run has_internet()
        so we do not block this node's main thread.
        """
        t = threading.Thread(target=self._check_internet_bg)
        t.daemon = True
        t.start()

    def _check_internet_bg(self):
        net_ok = has_internet()
        if net_ok:
            if self.internet_state != "INTERNET_OK":
                self.display.set_cellular_status(True)
                self.internet_state = "INTERNET_OK"
                self.get_logger().info("Internet state changed: INTERNET_OK")
        else:
            if self.internet_state != "INTERNET_NO_CONNECTION":
                self.display.set_cellular_status(False)
                self.internet_state = "INTERNET_NO_CONNECTION"
                self.get_logger().warn("Internet state changed: INTERNET_NO_CONNECTION")

    def check_time_sync(self):
        """
        Fire up a separate thread to run chrony_has_pps()
        so we don't block the main thread.
        """
        t = threading.Thread(target=self._check_time_sync_bg)
        t.daemon = True
        t.start()

    def _check_time_sync_bg(self):
        pps_ok = chrony_has_pps()
        if pps_ok:
            if self.time_sync_state != "TIME_SYNC_PPS":
                self.time_sync_state = "TIME_SYNC_PPS"
                self.check_full_synchronization()
                self.get_logger().info("Time Sync state changed: TIME_SYNC_PPS (PPS Reach=377)")
        else:
            if self.time_sync_state != "TIME_SYNC_NO_PPS":
                self.time_sync_state = "TIME_SYNC_NO_PPS"
                self.check_full_synchronization()
                self.get_logger().warn("Time Sync state changed: TIME_SYNC_NO_PPS (PPS not 377)")


def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()

    # Spin in a single thread, or use MultiThreadedExecutor if you prefer:
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()