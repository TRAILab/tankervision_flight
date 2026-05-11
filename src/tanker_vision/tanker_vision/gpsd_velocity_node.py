#!/usr/bin/env python3
"""
Reads ground speed from gpsd and publishes it as geometry_msgs/Vector3Stamped
on /filter/velocity so status_node can detect takeoff and landing.

gpsd must be running and have a GNSS fix.
Speed is published as (x=ground_speed_m_s, y=0, z=0).

Config key (under gpsd_velocity):
  publish_rate_hz: float  — max publish rate (default 1.0 Hz)
"""
import os
import time
import threading
import yaml
import gps

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped


def _deep_merge(base: dict, override: dict) -> dict:
    result = dict(base)
    for k, v in override.items():
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = _deep_merge(result[k], v)
        else:
            result[k] = v
    return result


def _load_config(node) -> dict:
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
                node.get_logger().error(f'Failed to load config {path}: {e}')
    return cfg


class GpsdVelocityNode(Node):
    def __init__(self):
        super().__init__('gpsd_velocity_node')
        self.declare_parameter('config', '')
        self.declare_parameter('unit_config', '')

        cfg = _load_config(self)
        gpsd_cfg = cfg.get('gpsd_velocity', {})
        self._min_interval = 1.0 / max(float(gpsd_cfg.get('publish_rate_hz', 1.0)), 0.01)

        self._pub = self.create_publisher(Vector3Stamped, '/filter/velocity', 10)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'gpsd_velocity_node started — publishing on /filter/velocity '
            f'at {1.0 / self._min_interval:.1f} Hz'
        )

    def _run(self):
        last_publish = 0.0
        while rclpy.ok():
            try:
                session = gps.gps(mode=gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
                for report in session:
                    if not rclpy.ok():
                        break
                    if report.get('class') != 'TPV':
                        continue
                    now = time.monotonic()
                    if now - last_publish < self._min_interval:
                        continue
                    last_publish = now
                    speed = float(report.get('speed', 0.0))
                    msg = Vector3Stamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'gpsd'
                    msg.vector.x = speed
                    self._pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f'gpsd connection lost: {e} — retrying in 5s')
                time.sleep(5)


def main(args=None):
    rclpy.init(args=args)
    node = GpsdVelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
