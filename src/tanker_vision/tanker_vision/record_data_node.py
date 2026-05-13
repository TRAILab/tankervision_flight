import os
import time
import yaml
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import numpy as np

_LATCHED_QOS = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


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
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            cfg = _deep_merge(cfg, data)
    return cfg


class RecordDataNode(Node):
    def __init__(self):
        super().__init__('record_data_node')

        self.declare_parameter('config', '')
        self.declare_parameter('unit_config', '')

        cfg         = _load_config(self)
        yolo_cfg    = cfg.get('yolo', {})
        trigger_cfg = cfg.get('trigger', {})

        # Resolve model path: relative paths are relative to repo root
        raw_model   = yolo_cfg.get('model', 'src/tanker_vision/tanker_vision/fire_detection.pt')
        config_path = self.get_parameter('config').get_parameter_value().string_value
        repo_root   = os.path.dirname(os.path.dirname(config_path)) if config_path else ''
        model_path  = (raw_model if os.path.isabs(raw_model)
                       else os.path.join(repo_root, raw_model))

        self._fire_class   = int(yolo_cfg.get('fire_class', 2))
        self._confidence   = float(yolo_cfg.get('confidence', 0.3))
        self._timeout_sec  = float(trigger_cfg.get('timeout_sec', 20.0))
        self._cooldown_sec = float(trigger_cfg.get('cooldown_sec', 5.0))

        self._session_path   = None
        self._is_recording   = False
        self._recording_end  = 0.0
        self._cooldown_until = 0.0
        self._process        = None

        self.yolo_model = YOLO(model_path, verbose=False)

        if not torch.cuda.is_available():
            raise RuntimeError(
                "CUDA is not available. Refusing to start RecordDataNode because YOLO trigger must run on GPU."
            )

        self.yolo_model.to("cuda")
        self._yolo_device = 0

        self.get_logger().info(
            f"YOLO using CUDA: {torch.cuda.get_device_name(0)} | "
            f"model device={next(self.yolo_model.model.parameters()).device}"
        )

        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        _ = self.yolo_model.predict(
            dummy,
            imgsz=640,
            conf=self._confidence,
            device=self._yolo_device,
            verbose=False,
        )
        torch.cuda.synchronize()
        self.get_logger().info("YOLO warmup complete")

        self.cv_bridge  = CvBridge()

        # Latched subscription — receives session path even if status_node started first
        self.create_subscription(
            String, '/tankervision/session_path', self._session_path_cb, _LATCHED_QOS)

        self.create_subscription(Image, '/cam0/image_raw', self._image_cb, 3)

        self._status_pub  = self.create_publisher(String, '/record_data/status',  10)
        self._trigger_pub = self.create_publisher(Empty,  '/save_images_trigger', 10)

        self.create_timer(1.0, self._check_recording_status)
        self.get_logger().info(
            f'RecordDataNode started | model={model_path} '
            f'conf={self._confidence} class={self._fire_class}'
        )

    def _session_path_cb(self, msg: String):
        self._session_path = msg.data
        self.get_logger().info(f'Session path: {self._session_path}')

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def _image_cb(self, msg: Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.yolo_model.predict(
            cv_image,
            imgsz=640,
            conf=self._confidence,
            device=self._yolo_device,
            verbose=False,
        )

        fire_detected = False
        if results and hasattr(results[0], 'boxes'):
            for box in results[0].boxes:
                if int(box.cls[0]) == self._fire_class:
                    fire_detected = True
                    break

        if fire_detected:
            self._trigger_pub.publish(Empty())
            self._start_or_extend_recording()

        if not self._is_recording:
            self._publish_status('SCANNING')
        elif self._process and self._process.poll() is None:
            self._publish_status('RECORDING')

    def _start_or_extend_recording(self):
        now = time.time()

        if now < self._cooldown_until:
            return

        if self._is_recording:
            self._recording_end = now + self._timeout_sec
            return

        if self._session_path is None:
            self.get_logger().warn('Fire detected but session path not yet received — skipping')
            return

        ts       = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        bag_path = os.path.join(self._session_path, f'fire_{ts}')

        cmd = [
            '/opt/ros/humble/bin/ros2', 'bag', 'record',
            '-o', bag_path,
            '/cam0/image_raw', '/im19/imu',
            '--compression-mode', 'message',
            '--compression-format', 'zstd',
            '-b', '100000000',
        ]
        self._process       = subprocess.Popen(cmd)
        self._is_recording  = True
        self._recording_end = now + self._timeout_sec
        self.get_logger().info(f'Recording started: {bag_path}')

    def _check_recording_status(self):
        self._publish_status('RECORDING' if self._is_recording else 'SCANNING')
        if not self._is_recording:
            return
        if time.time() > self._recording_end:
            if self._process is not None:
                self._process.terminate()
                self._process.wait()
                self._process = None
            self._is_recording   = False
            self._cooldown_until = time.time() + self._cooldown_sec
            self.get_logger().info('Recording stopped.')

    def destroy_node(self):
        if self._process is not None:
            self._process.terminate()
            self._process.wait()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecordDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
