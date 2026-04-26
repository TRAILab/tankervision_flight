#!/usr/bin/env python3

import os
import time
import struct
from pathlib import Path

from datetime import datetime, timezone, timedelta
from builtin_interfaces.msg import Time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

MEMS_HEADER  = b"fmim"
NAVI_HEADER  = b"fmin"
COMMON_PREFIX = b"fmi"
MEMS_LEN = 52
NAVI_LEN = 100
TAIL     = b"ed"
FMT_MEMS = "<4sd9fH2s"
G_TO_MPS2 = 9.80665

def utc_hhmmss_to_ros_time(utc_raw: float) -> Time:
    """
    Convert IM19 UTC time-of-day in hhmmss.ss format into a ROS Time message.

    Example:
        193113.76550619243 -> today at 19:31:13.765506192 UTC

    Because the IMU gives no date, this uses the Jetson's current UTC date.
    """
    utc_now = datetime.now(timezone.utc)

    hhmmss_int = int(utc_raw)
    frac = float(utc_raw) - hhmmss_int

    hour = hhmmss_int // 10000
    minute = (hhmmss_int // 100) % 100
    second = hhmmss_int % 100

    # nanoseconds from fractional seconds
    nanosec = int(round(frac * 1_000_000_000))

    # handle possible rounding edge case, e.g. .9999999996 rounds to 1e9
    if nanosec >= 1_000_000_000:
        second += 1
        nanosec -= 1_000_000_000

    stamp_dt = utc_now.replace(
        hour=hour,
        minute=minute,
        second=second,
        microsecond=0,
    )

    # Handle midnight rollover if Jetson UTC date and IMU UTC time-of-day
    # are on opposite sides of midnight.
    delta = stamp_dt - utc_now
    if delta > timedelta(hours=12):
        stamp_dt -= timedelta(days=1)
    elif delta < timedelta(hours=-12):
        stamp_dt += timedelta(days=1)

    sec = int(stamp_dt.timestamp())

    return Time(sec=sec, nanosec=nanosec)


def parse_mems_packet(packet: bytes):
    if len(packet) != MEMS_LEN:
        return None
    try:
        values = struct.unpack(FMT_MEMS, packet)
    except struct.error:
        return None
    (header, utc_raw, acc_x_g, acc_y_g, acc_z_g,
     gyro_x_rps, gyro_y_rps, gyro_z_rps,
     _, _, _, checksum_recv, tail) = values
    if header != MEMS_HEADER or tail != TAIL:
        return None
    if (sum(packet[:48]) & 0xFFFF) != checksum_recv:
        return None
    return {
        "utc_raw":    utc_raw,
        "acc_x_g":    acc_x_g,   "acc_y_g":    acc_y_g,   "acc_z_g":    acc_z_g,
        "acc_x_mps2": acc_x_g * G_TO_MPS2,
        "acc_y_mps2": acc_y_g * G_TO_MPS2,
        "acc_z_mps2": acc_z_g * G_TO_MPS2,
        "gyro_x_rps": gyro_x_rps, "gyro_y_rps": gyro_y_rps, "gyro_z_rps": gyro_z_rps,
    }


class CombinedExtractor:
    def __init__(self):
        self.buf           = bytearray()
        self.tail_fail     = 0
        self.checksum_fail = 0
        self.header_skip   = 0

    def feed(self, chunk: bytes):
        self.buf.extend(chunk)

    def get_one(self):
        while True:
            idx = self.buf.find(COMMON_PREFIX)
            if idx < 0:
                if len(self.buf) > 2:
                    self.header_skip += max(0, len(self.buf) - 2)
                    del self.buf[:-2]
                return None

            if idx > 0:
                self.header_skip += idx
                del self.buf[:idx]

            if len(self.buf) < 4:
                return None

            fourth = self.buf[3]
            if fourth == ord('m'):
                pkt_type, pkt_len, cs_end = 'mems', MEMS_LEN, 48
            elif fourth == ord('n'):
                pkt_type, pkt_len, cs_end = 'navi', NAVI_LEN, 96
            else:
                self.header_skip += 1
                del self.buf[0]
                continue

            if len(self.buf) < pkt_len:
                return None

            candidate = bytes(self.buf[:pkt_len])

            if candidate[-2:] != TAIL:
                self.tail_fail += 1
                del self.buf[0]
                continue

            calc = sum(candidate[:cs_end]) & 0xFFFF
            recv = struct.unpack_from('<H', candidate, cs_end)[0]
            if calc != recv:
                self.checksum_fail += 1
                del self.buf[0]
                continue

            del self.buf[:pkt_len]
            return candidate, pkt_type


class IM19MemsNode(Node):
    def __init__(self):
        super().__init__("im19_mems_node")

        self.declare_parameter("port",     "/dev/im19_mems")
        self.declare_parameter("baud",     115200)
        self.declare_parameter("timeout",  0.05)
        self.declare_parameter("frame_id", "im19")
        self.declare_parameter("output_dir", str(Path.home() / "im19_logs"))
        self.declare_parameter("chunk_size", 1024)
        self.declare_parameter("stats_period_sec",    300.0)
        self.declare_parameter("flush_period_sec",      5.0)
        self.declare_parameter("reconnect_period_sec",  1.0)
        self.declare_parameter("send_startup_commands", True)
        self.declare_parameter("startup_commands", ["AT+MEMS_OUTPUT=UART1,ON"])

        self.port      = self.get_parameter("port").value
        self.baud      = int(self.get_parameter("baud").value)
        self.timeout   = float(self.get_parameter("timeout").value)
        self.frame_id  = self.get_parameter("frame_id").value
        self.chunk_size = int(self.get_parameter("chunk_size").value)
        self.stats_period_sec    = float(self.get_parameter("stats_period_sec").value)
        self.flush_period_sec    = float(self.get_parameter("flush_period_sec").value)
        self.reconnect_period_sec = float(self.get_parameter("reconnect_period_sec").value)
        self.send_startup_commands = bool(self.get_parameter("send_startup_commands").value)
        self.startup_commands = list(self.get_parameter("startup_commands").value)

        self._output_dir_base = self.get_parameter("output_dir").value
        self.output_dir  = self._output_dir_base
        self.mems_file   = None
        self.navi_file   = None

        self.ser = None
        self.connected = False
        self.last_reconnect_try = 0.0

        self.extractor   = CombinedExtractor()
        self.good_mems   = 0
        self.good_navi   = 0

        self.imu_pub = self.create_publisher(Imu,              "/im19/imu",     50)
        self.raw_pub = self.create_publisher(Float64MultiArray, "/im19/rawdata", 50)

        self.create_timer(0.001,                   self.read_serial_once)
        self.create_timer(self.stats_period_sec,   self.print_stats)
        self.create_timer(self.flush_period_sec,   self.flush_files)

        self.try_open_serial(initial=True)

    def _make_session_dir(self) -> bool:
        if time.localtime().tm_year < 2020:
            return False
        now = time.localtime()
        self.output_dir = os.path.join(
            self._output_dir_base,
            time.strftime("%Y-%m-%d", now),
            time.strftime("imu_%H-%M-%S", now),
        )
        os.makedirs(self.output_dir, exist_ok=True)
        return True

    def _ensure_mems_file(self):
        if self.mems_file is not None:
            return
        if self.output_dir == self._output_dir_base:
            if not self._make_session_dir():
                return
        path = os.path.join(self.output_dir, f"im19_mems_raw_{time.strftime('%Y%m%d_%H%M%S')}.bin")
        self.mems_file = open(path, "ab")
        self.get_logger().info(f"MEMS output: {path}")

    def _ensure_navi_file(self):
        if self.navi_file is not None:
            return
        if self.output_dir == self._output_dir_base:
            if not self._make_session_dir():
                return
        path = os.path.join(self.output_dir, f"im19_navi_raw_{time.strftime('%Y%m%d_%H%M%S')}.bin")
        self.navi_file = open(path, "ab")
        self.get_logger().info(f"NAVI output: {path}")

    def destroy_node(self):
        for f in (self.mems_file, self.navi_file):
            try:
                if f is not None:
                    f.flush()
                    f.close()
            except Exception:
                pass
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
        if (not initial) and (now - self.last_reconnect_try < self.reconnect_period_sec):
            return
        self.last_reconnect_try = now
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=self.timeout)
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
                time.sleep(0.2)
                response = self.ser.read(1024)
                if response:
                    preview = response.decode(errors="replace").strip()
                    if preview:
                        self.get_logger().info(f"Startup [{cmd}]: {preview[:200]}")
                    else:
                        self.get_logger().info(f"Startup sent: {cmd}")
                else:
                    self.get_logger().info(f"Startup sent: {cmd}")
            except Exception as e:
                self.get_logger().error(f"Failed to send startup [{cmd}]: {e}")
                self.close_serial()
                return

    def publish_rawdata(self, data: dict):
        msg = Float64MultiArray()
        msg.data = [
            float(data["utc_raw"]),
            float(data["acc_x_g"]),   float(data["acc_y_g"]),   float(data["acc_z_g"]),
            float(data["gyro_x_rps"]), float(data["gyro_y_rps"]), float(data["gyro_z_rps"]),
        ]
        self.raw_pub.publish(msg)

    def publish_imu(self, data: dict):
        msg = Imu()
        msg.header.stamp    = utc_hhmmss_to_ros_time(float(data["utc_raw"]))
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance[0]         = -1.0
        msg.angular_velocity_covariance[0]    = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        msg.angular_velocity.x = float(data["gyro_x_rps"])
        msg.angular_velocity.y = float(data["gyro_y_rps"])
        msg.angular_velocity.z = float(data["gyro_z_rps"])
        msg.linear_acceleration.x = float(data["acc_x_mps2"])
        msg.linear_acceleration.y = float(data["acc_y_mps2"])
        msg.linear_acceleration.z = float(data["acc_z_mps2"])
        self.imu_pub.publish(msg)

    def flush_files(self):
        for f in (self.mems_file, self.navi_file):
            if f is None:
                continue
            try:
                f.flush()
            except Exception as e:
                self.get_logger().error(f"Flush error: {e}")

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
            result = self.extractor.get_one()
            if result is None:
                break
            pkt, pkt_type = result

            if pkt_type == 'mems':
                self._ensure_mems_file()
                try:
                    self.mems_file.write(pkt)
                except Exception as e:
                    self.get_logger().error(f"MEMS write error: {e}")
                data = parse_mems_packet(pkt)
                if data:
                    self.good_mems += 1
                    self.publish_rawdata(data)
                    self.publish_imu(data)

            elif pkt_type == 'navi':
                self._ensure_navi_file()
                try:
                    self.navi_file.write(pkt)
                except Exception as e:
                    self.get_logger().error(f"NAVI write error: {e}")
                self.good_navi += 1

    def print_stats(self):
        self.get_logger().info(
            f"mems={self.good_mems} navi={self.good_navi} "
            f"tail_fail={self.extractor.tail_fail} "
            f"checksum_fail={self.extractor.checksum_fail} "
            f"header_skip={self.extractor.header_skip} "
            f"connected={self.connected}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = IM19MemsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
