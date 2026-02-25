#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import TimeReference
from std_msgs.msg import String
import serial
import datetime

def nmea_checksum(sentence_body: str) -> int:
    """
    Compute the NMEA 0183 checksum for the sentence body (the part after '$' but before '*').
    """
    csum = 0
    for ch in sentence_body:
        csum ^= ord(ch)
    return csum

class NmeaToSerialLifecycleNode(Node):
    def __init__(self):
        super().__init__('nmea_and_utc_to_serial_node')

        # Create a publisher for heartbeat messages.
        self.heartbeat_pub = self.create_publisher(String, 'gps_heartbeat', 10)
        
        # Create subscriptions for incoming NMEA sentences and UTC time.
        self.nmea_sub = self.create_subscription(
            Sentence,
            '/nmea',
            self.nmea_callback,
            10
        )
        self.utctime_sub = self.create_subscription(
            TimeReference,
            '/imu/utctime',
            self.utctime_callback,
            10
        )

        # To build ZDA messages, we need the current date.
        self.current_date = None

        # Open the serial port (adjust device and baudrate as needed).
        try:
            self.serial_port = serial.Serial('/dev/ttyV0', 9600, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None

        self.get_logger().info('NMEA and UTC to Serial Lifecycle Node started.')

    def utctime_callback(self, msg: TimeReference):
        """
        Update the current date from the /imu/utctime topic.
        """
        unix_time = msg.time_ref.sec + msg.time_ref.nanosec * 1e-9
        dt = datetime.datetime.utcfromtimestamp(unix_time)
        self.current_date = dt.date()
        self.get_logger().debug(f"Updated date from utctime: {self.current_date}")

    def nmea_callback(self, msg: Sentence):
        """
        Process incoming NMEA sentences. If the sentence is a GGA-type and its time
        field indicates an integer second, forward it and generate a ZDA sentence.
        """
        nmea_line = msg.sentence.strip()

        # Process only GGA (or similar) sentences.
        if nmea_line.startswith(("$GPGGA", "$GNGGA", "$GAGGA")):
            fields = nmea_line.split(',')
            if len(fields) < 2:
                return  # Malformed sentence

            # GGA time field is at index 1, e.g. "185213.00"
            gga_time_str = fields[1].strip()
            try:
                gga_time_val = float(gga_time_str)
                fractional = gga_time_val - int(gga_time_val)
            except ValueError:
                return  # Not a valid float

            # If the fractional part is near zero, assume an integer second.
            if abs(fractional) < 1e-4:
                # Publish a heartbeat message @ 1Hz.
                heartbeat_msg = String()
                heartbeat_msg.data = "heartbeat"
                self.heartbeat_pub.publish(heartbeat_msg)
                # Forward the GGA sentence.
                self.get_logger().debug(f"Publishing GGA: {nmea_line}")
                self.write_to_serial(nmea_line)
                # Generate and publish a ZDA sentence.
                self.publish_zda(gga_time_val)

    def publish_zda(self, gga_time_val: float):
        """
        Build and send a $GPZDA sentence using:
         - Time from the GGA sentence.
         - Date from the /imu/utctime message.
         - Zero offsets for time zone.
        """
        if not self.current_date:
            self.get_logger().warn("No /imu/utctime date yet; skipping ZDA generation.")
            return

        hours = int(gga_time_val // 10000)
        remainder = gga_time_val % 10000  # e.g., 5213.00
        minutes = int(remainder // 100)
        seconds = remainder % 100

        # Format the time as hhmmss.ss (removing trailing zeros if needed).
        time_str = f"{hours:02d}{minutes:02d}{seconds:06.3f}".rstrip('0').rstrip('.')
        
        # Construct the ZDA body: GPZDA,hhmmss.ss,DD,MM,YYYY,00,00
        zda_body = (
            f"GPZDA,{time_str},"
            f"{self.current_date.day:02d},"
            f"{self.current_date.month:02d},"
            f"{self.current_date.year},00,00"
        )
        csum = nmea_checksum(zda_body)
        zda_sentence = f"${zda_body}*{csum:02X}\r\n"

        self.get_logger().debug(f"Publishing ZDA: {zda_sentence.strip()}")
        self.write_to_serial(zda_sentence.strip())

    def write_to_serial(self, sentence: str):
        """
        Write a sentence to the serial port (adding the required CR/LF) 
        """
        full_line = sentence + "\r\n"
        self.get_logger().debug(f"Serial Out: {full_line.strip()}")

        if self.serial_port:
            try:
                self.serial_port.write(full_line.encode('ascii', errors='ignore'))
                self.serial_port.flush()
            except serial.SerialException as e:
                self.get_logger().error(f"Error writing to serial port: {e}")
        else:
            self.get_logger().error("Serial port not available.")

def main(args=None):
    rclpy.init(args=args)
    node = NmeaToSerialLifecycleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()