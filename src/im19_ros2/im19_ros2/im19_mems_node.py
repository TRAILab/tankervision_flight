#!/usr/bin/env python3

import os
import time
import struct
from pathlib import Path

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

PACKET_LEN = 52
HEADER = b"fmim"
TAIL = b"ed"
FMT_MEMS = "<4sd9fH2s"
G_TO_MPS2 = 9.80665


def checksum_1_to_48(packet: bytes) -> int:
    return sum(packet[:48]) & 0xFFFF


def parse_mems_packet(packet: bytes):
    if len(packet) != PACKET_LEN:
        return None

    try:
        values = struct.unpack(FMT_MEMS, packet)
    except struct.error:
        return None

    (
        header,
        utc_raw,
        acc_x_g,
        acc_y_g,
        acc_z_g,
        gyro_x_rps,
        gyro_y_rps,
        gyro_z_rps,
        reserved1,
        reserved2,
        reserved3,
        checksum_recv,
        tail,
    ) = values

    if header != HEADER:
        return None
    if tail != TAIL:
        return None

    checksum_calc = checksum_1_to_48(packet)
    if checksum_calc != checksum_recv:
        return None

    return {
        "utc_raw": utc_raw,
        "acc_x_g": acc_x_g,
        "acc_y_g": acc_y_g,
        "acc_z_g": acc_z_g,
        "acc_x_mps2": acc_x_g * G_TO_MPS2,
        "acc_y_mps2": acc_y_g * G_TO_MPS2,
        "acc_z_mps2": acc_z_g * G_TO_MPS2,
        "gyro_x_rps": gyro_x_rps,
        "gyro_y_rps": gyro_y_rps,
        "gyro_z_rps": gyro_z_rps,
        "reserved1": reserved1,
        "reserved2": reserved2,
        "reserved3": reserved3,
        "checksum_recv": checksum_recv,
        "checksum_calc": checksum_calc,
    }


class MemsPacketExtractor:
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

            if candidate[-2:] != TAIL:
                self.tail_fail += 1
                del self.buf[0]
                continue

            parsed = parse_mems_packet(candidate)
            if parsed is None:
                self.checksum_fail += 1
                del self.buf[0]
                continue

            del self.buf[:PACKET_LEN]
            return candidate


class IM19MemsNode(Node):
    def __init__(self):
        super().__init__("im19_mems_node")

        self.declare_parameter("port", "/dev/ttyUSB2")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("timeout", 0.05)
        self.declare_parameter("frame_id", "im19")
        self.declare_parameter("output_dir", str(Path.home() / "im19_logs"))
        self.declare_parameter("chunk_size", 1024)

        self.declare_parameter("stats_period_sec", 300.0)
        self.declare_parameter("flush_period_sec", 5.0)
        self.declare_parameter("reconnect_period_sec", 1.0)

        self.declare_parameter("send_startup_commands", True)
        self.declare_parameter("startup_commands", ["AT+MEMS_OUTPUT=UART1,ON"])

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.output_dir = self.get_parameter("output_dir").value
        self.chunk_size = int(self.get_parameter("chunk_size").value)

        self.stats_period_sec = float(self.get_parameter("stats_period_sec").value)
        self.flush_period_sec = float(self.get_parameter("flush_period_sec").value)
        self.reconnect_period_sec = float(self.get_parameter("reconnect_period_sec").value)

        self.send_startup_commands = bool(self.get_parameter("send_startup_commands").value)
        self.startup_commands = list(self.get_parameter("startup_commands").value)

        os.makedirs(self.output_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.bin_path = os.path.join(self.output_dir, f"im19_mems_raw_{ts}.bin")
        self.bin_file = open(self.bin_path, "ab")

        self.ser = None
        self.connected = False
        self.last_reconnect_try = 0.0

        self.extractor = MemsPacketExtractor()
        self.good_packets = 0

        self.imu_pub = self.create_publisher(Imu, "/im19/imu", 50)
        self.raw_pub = self.create_publisher(Float64MultiArray, "/im19/rawdata", 50)

        self.timer = self.create_timer(0.001, self.read_serial_once)
        self.stat_timer = self.create_timer(self.stats_period_sec, self.print_stats)
        self.flush_timer = self.create_timer(self.flush_period_sec, self.flush_file)

        self.try_open_serial(initial=True)

    def destroy_node(self):
        try:
            self.bin_file.flush()
            self.bin_file.close()
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
                time.sleep(0.2)

                response = self.ser.read(1024)
                if response:
                    preview = response.decode(errors="replace").strip()
                    if preview:
                        self.get_logger().info(f"Startup command [{cmd}] response: {preview[:200]}")
                    else:
                        self.get_logger().info(f"Startup command sent: {cmd}")
                else:
                    self.get_logger().info(f"Startup command sent: {cmd}")
            except Exception as e:
                self.get_logger().error(f"Failed to send startup command [{cmd}]: {e}")
                self.close_serial()
                return

    def publish_rawdata(self, data: dict):
        msg = Float64MultiArray()
        msg.data = [
            float(data["utc_raw"]),
            float(data["acc_x_g"]),
            float(data["acc_y_g"]),
            float(data["acc_z_g"]),
            float(data["gyro_x_rps"]),
            float(data["gyro_y_rps"]),
            float(data["gyro_z_rps"]),
        ]
        self.raw_pub.publish(msg)

    def publish_imu(self, data: dict):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = float(data["gyro_x_rps"])
        msg.angular_velocity.y = float(data["gyro_y_rps"])
        msg.angular_velocity.z = float(data["gyro_z_rps"])

        msg.linear_acceleration.x = float(data["acc_x_mps2"])
        msg.linear_acceleration.y = float(data["acc_y_mps2"])
        msg.linear_acceleration.z = float(data["acc_z_mps2"])

        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(msg)

    def flush_file(self):
        try:
            self.bin_file.flush()
        except Exception as e:
            self.get_logger().error(f"Failed to flush bin file: {e}")

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

        try:
            self.bin_file.write(chunk)
        except Exception as e:
            self.get_logger().error(f"Failed to write bin file: {e}")

        self.extractor.feed(chunk)

        while True:
            pkt = self.extractor.get_one()
            if pkt is None:
                break

            data = parse_mems_packet(pkt)
            if data is None:
                continue

            self.good_packets += 1
            self.publish_rawdata(data)
            self.publish_imu(data)

    def print_stats(self):
        self.get_logger().info(
            f"good_packets={self.good_packets}, "
            f"tail_fail={self.extractor.tail_fail}, "
            f"checksum_fail={self.extractor.checksum_fail}, "
            f"header_skip={self.extractor.header_skip}, "
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
