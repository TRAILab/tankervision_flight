#!/usr/bin/env python3

import math
import time
import struct

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32, Float64MultiArray

# ── NAVI binary packet spec (IM19EE Integration Guide v1.4.1, section 3.1.3) ──
# Total: 100 bytes, little-endian
# Bytes 1-4:   header  'f','m','i','n'
# Bytes 5-12:  Field1  hhmmss.ss   double
# Bytes 13-20: Field2  latitude    double  (degrees)
# Bytes 21-28: Field3  longitude   double  (degrees)
# Bytes 29-36: Field4  height      double  (metres)
# Bytes 37-40: Field5  N velocity  float   (m/s)
# Bytes 41-44: Field6  E velocity  float   (m/s)
# Bytes 45-48: Field7  D velocity  float   (m/s)
# Bytes 49-52: Field8  roll        float   (rad)
# Bytes 53-56: Field9  pitch       float   (rad)
# Bytes 57-60: Field10 yaw         float   (rad)
# Bytes 61-64: Field11 pos quality float
# Bytes 65-68: Field12 acc_x bias  float   (m/s^2)
# Bytes 69-72: Field13 acc_y bias  float   (m/s^2)
# Bytes 73-76: Field14 acc_z bias  float   (m/s^2)
# Bytes 77-80: Field15 gyro_x bias float   (rad/s)
# Bytes 81-84: Field16 gyro_y bias float   (rad/s)
# Bytes 85-88: Field17 gyro_z bias float   (rad/s)
# Bytes 89-92: Field18 sensor temp float   (degC)
# Bytes 93-96: Field19 STATUS      uint32
# Bytes 97-98: checksum            uint16  (sum of bytes 1-96)
# Bytes 99-100: tail   'e','d'

PACKET_LEN = 100
HEADER = b"fmin"
TAIL = b"ed"

# STATUS bit definitions
STATUS_FINIT = 0x00000001
STATUS_READY = 0x00000002
STATUS_INACCURATE = 0x00000004
STATUS_GNSS_REJECT = 0x00000010
STATUS_FRESET = 0x00000020
STATUS_PPS_READY = 0x00040000
STATUS_SYNC_READY = 0x00080000
STATUS_GNSS_CONNECT = 0x00100000

# Sentinel value used by IM19 when a field is not yet valid
SENTINEL_YAW = 9.99999


def checksum_navi(packet: bytes) -> int:
    """Sum bytes 0..95 (positions 1-96 in 1-indexed spec), masked to uint16."""
    return sum(packet[:96]) & 0xFFFF


def parse_navi_packet(packet: bytes):
    if len(packet) != PACKET_LEN:
        return None
    if packet[:4] != HEADER:
        return None
    if packet[98:100] != TAIL:
        return None

    calc = checksum_navi(packet)
    recv = struct.unpack_from("<H", packet, 96)[0]
    if calc != recv:
        return None

    (
        utc,
        lat, lon, alt,
        vel_n, vel_e, vel_d,
        roll, pitch, yaw,
        pos_quality,
        acc_x_bias, acc_y_bias, acc_z_bias,
        gyro_x_bias, gyro_y_bias, gyro_z_bias,
        temperature,
    ) = struct.unpack_from("<d3d3f3f1f3f3ff", packet, 4)

    status = struct.unpack_from("<I", packet, 92)[0]

    return {
        "utc": utc,
        "lat_deg": lat,
        "lon_deg": lon,
        "alt_m": alt,
        "vel_n": vel_n,
        "vel_e": vel_e,
        "vel_d": vel_d,
        "roll_rad": roll,
        "pitch_rad": pitch,
        "yaw_rad": yaw,
        "pos_quality": pos_quality,
        "acc_x_bias": acc_x_bias,
        "acc_y_bias": acc_y_bias,
        "acc_z_bias": acc_z_bias,
        "gyro_x_bias": gyro_x_bias,
        "gyro_y_bias": gyro_y_bias,
        "gyro_z_bias": gyro_z_bias,
        "temperature": temperature,
        "status": status,
        "ready": bool(status & STATUS_READY),
        "sync_ready": bool(status & STATUS_SYNC_READY),
        "gnss_connect": bool(status & STATUS_GNSS_CONNECT),
        "pps_ready": bool(status & STATUS_PPS_READY),
        "yaw_valid": abs(yaw - 9.99999) > 0.001,
        "position_valid": (lat != 0.0 or lon != 0.0),
    }


def euler_to_quaternion(roll, pitch, yaw):
    """Convert roll/pitch/yaw (radians) to quaternion (x, y, z, w)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class NaviPacketExtractor:
    def __init__(self):
        self.buf = bytearray()
        self.tail_fail = 0
        self.checksum_fail = 0
        self.header_skip = 0

    def feed(self, chunk: bytes):
        self.buf.extend(chunk)

    def get_one(self):
        while True:
            idx = self.buf.find(HEADER)
            if idx < 0:
                if len(self.buf) > 3:
                    self.header_skip += max(0, len(self.buf) - 3)
                    del self.buf[:-3]
                return None

            if idx > 0:
                self.header_skip += idx
                del self.buf[:idx]

            if len(self.buf) < PACKET_LEN:
                return None

            candidate = bytes(self.buf[:PACKET_LEN])

            if candidate[98:100] != TAIL:
                self.tail_fail += 1
                del self.buf[0]
                continue

            parsed = parse_navi_packet(candidate)
            if parsed is None:
                self.checksum_fail += 1
                del self.buf[0]
                continue

            del self.buf[:PACKET_LEN]
            return candidate


class IM19NaviNode(Node):
    def __init__(self):
        super().__init__("im19_navi_node")  # matches im19_mems_node convention

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter("port", "/dev/im19_navi")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("timeout", 0.05)
        self.declare_parameter("frame_id", "im19")
        self.declare_parameter("chunk_size", 1024)
        self.declare_parameter("stats_period_sec", 300.0)
        self.declare_parameter("reconnect_period_sec", 1.0)
        self.declare_parameter("send_startup_commands", True)
        self.declare_parameter("startup_commands", ["AT+NAVI_OUTPUT=UART1,ON"])

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.chunk_size = int(self.get_parameter("chunk_size").value)
        self.stats_period_sec = float(self.get_parameter("stats_period_sec").value)
        self.reconnect_period_sec = float(
            self.get_parameter("reconnect_period_sec").value
        )
        self.send_startup_commands = bool(
            self.get_parameter("send_startup_commands").value
        )
        self.startup_commands = list(self.get_parameter("startup_commands").value)

        # ── Publishers ────────────────────────────────────────────
        # Primary fused pose — use this for navigation
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/im19/pose", 10
        )
        # Full odometry (pose + velocity)
        self.odom_pub = self.create_publisher(Odometry, "/im19/odom", 10)
        # Velocity in body frame
        self.twist_pub = self.create_publisher(
            TwistWithCovarianceStamped, "/im19/twist", 10
        )
        # Raw NAVI fields for debugging/logging
        self.raw_pub = self.create_publisher(
            Float64MultiArray, "/im19/navi_raw", 10
        )
        # STATUS word for monitoring filter state
        self.status_pub = self.create_publisher(UInt32, "/im19/navi_status", 10)

        # ── State ─────────────────────────────────────────────────
        self.ser = None
        self.connected = False
        self.last_reconnect_try = 0.0
        self.extractor = NaviPacketExtractor()
        self.good_packets = 0
        self.last_status = None

        # ── Timers ────────────────────────────────────────────────
        self.timer = self.create_timer(0.001, self.read_serial_once)
        self.stat_timer = self.create_timer(self.stats_period_sec, self.print_stats)

        self.try_open_serial(initial=True)
        self.get_logger().info(
            f"IM19 NAVI node started on {self.port} @ {self.baud}"
        )

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()

    def close_serial(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connected = False

    def try_open_serial(self, initial=False):
        now = time.time()
        if (not initial) and (
            now - self.last_reconnect_try < self.reconnect_period_sec
        ):
            return
        self.last_reconnect_try = now

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout,
            )
            self.connected = True
            self.get_logger().info(f"Opened serial {self.port} @ {self.baud}")
            if self.send_startup_commands:
                self.send_commands(self.startup_commands)
        except Exception as e:
            self.connected = False
            self.ser = None
            self.get_logger().error(f"Failed to open serial {self.port}: {e}")

    def send_commands(self, commands):
        if self.ser is None:
            return
        for cmd in commands:
            try:
                self.ser.reset_input_buffer()
                self.ser.write((cmd + "\r\n").encode())
                self.ser.flush()
                time.sleep(0.3)
                response = self.ser.read(1024)
                if response:
                    preview = response.decode(errors="replace").strip()
                    if preview:
                        self.get_logger().info(
                            f"Startup [{cmd}] response: {preview[:100]}"
                        )
                else:
                    self.get_logger().info(f"Startup command sent: {cmd}")
            except Exception as e:
                self.get_logger().error(
                    f"Failed to send startup command [{cmd}]: {e}"
                )
                self.close_serial()
                return

    def publish_navi(self, data: dict):
        stamp = self.get_clock().now().to_msg()

        # ── Quaternion from Euler ─────────────────────────────────
        qx, qy, qz, qw = euler_to_quaternion(
            data["roll_rad"], data["pitch_rad"],
            data["yaw_rad"] if data["yaw_valid"] else 0.0,
        )

        # ── PoseWithCovarianceStamped ─────────────────────────────
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.pose.position.x = data["lon_deg"]
        pose_msg.pose.pose.position.y = data["lat_deg"]
        pose_msg.pose.pose.position.z = data["alt_m"]
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw
        # Set covariance diagonal — -1 signals unknown for invalid fields
        if not data["position_valid"]:
            pose_msg.pose.covariance[0] = -1.0  # x unknown
            pose_msg.pose.covariance[7] = -1.0  # y unknown
        if not data["yaw_valid"]:
            pose_msg.pose.covariance[35] = -1.0  # yaw unknown
        self.pose_pub.publish(pose_msg)

        # ── Odometry ─────────────────────────────────────────────
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = self.frame_id
        odom_msg.pose.pose = pose_msg.pose.pose
        odom_msg.pose.covariance = pose_msg.pose.covariance
        # NED velocity → body frame (approximate)
        odom_msg.twist.twist.linear.x = float(data["vel_n"])
        odom_msg.twist.twist.linear.y = float(data["vel_e"])
        odom_msg.twist.twist.linear.z = float(data["vel_d"])
        self.odom_pub.publish(odom_msg)

        # ── TwistWithCovarianceStamped ────────────────────────────
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = stamp
        twist_msg.header.frame_id = self.frame_id
        twist_msg.twist.twist.linear.x = float(data["vel_n"])
        twist_msg.twist.twist.linear.y = float(data["vel_e"])
        twist_msg.twist.twist.linear.z = float(data["vel_d"])
        self.twist_pub.publish(twist_msg)

        # ── Raw fields ────────────────────────────────────────────
        raw_msg = Float64MultiArray()
        raw_msg.data = [
            float(data["utc"]),
            float(data["lat_deg"]),
            float(data["lon_deg"]),
            float(data["alt_m"]),
            float(data["vel_n"]),
            float(data["vel_e"]),
            float(data["vel_d"]),
            float(data["roll_rad"]),
            float(data["pitch_rad"]),
            float(data["yaw_rad"]),
            float(data["pos_quality"]),
            float(data["temperature"]),
            float(data["status"]),
        ]
        self.raw_pub.publish(raw_msg)

        # ── STATUS word ───────────────────────────────────────────
        status_msg = UInt32()
        status_msg.data = int(data["status"])
        self.status_pub.publish(status_msg)

        # ── Log status changes ────────────────────────────────────
        if data["status"] != self.last_status:
            self.get_logger().info(
                f"NAVI STATUS=0x{data['status']:08X} "
                f"ready={data['ready']} "
                f"sync={data['sync_ready']} "
                f"pps={data['pps_ready']} "
                f"gnss={data['gnss_connect']}"
            )
            self.last_status = data["status"]

    def read_serial_once(self):
        if not self.connected or self.ser is None:
            self.try_open_serial()
            return

        try:
            chunk = self.ser.read(self.chunk_size)
        except Exception as e:
            self.get_logger().error(f"Serial read failed: {e}")
            self.close_serial()
            return

        if not chunk:
            return

        self.extractor.feed(chunk)

        while True:
            pkt = self.extractor.get_one()
            if pkt is None:
                break
            data = parse_navi_packet(pkt)
            if data is None:
                continue
            self.good_packets += 1
            self.publish_navi(data)

    def print_stats(self):
        self.get_logger().info(
            f"good_packets={self.good_packets} "
            f"tail_fail={self.extractor.tail_fail} "
            f"checksum_fail={self.extractor.checksum_fail} "
            f"header_skip={self.extractor.header_skip} "
            f"connected={self.connected}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = IM19NaviNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()