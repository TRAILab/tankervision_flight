import os
import time
import yaml

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

        self._target_class = int(yolo_cfg.get('target_class', yolo_cfg.get('fire_class', 2)))
        self._confidence   = float(yolo_cfg.get('confidence', 0.3))
        self._timeout_sec  = float(trigger_cfg.get('timeout_sec', 20.0))

        self._is_recording   = False
        self._recording_end  = 0.0

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

        self.create_subscription(Image, '/cam0/image_raw', self._image_cb, 3)

        self._status_pub  = self.create_publisher(String, '/record_data/status',  10)
        self._trigger_pub = self.create_publisher(Empty,  '/save_images_trigger', 10)
        self._camera_record_mode_pub = self.create_publisher(
            String, '/camera/record_mode', _LATCHED_QOS)

        self.create_timer(1.0, self._check_recording_status)
        self.create_timer(5.0, self._publish_record_heartbeat)
        self.create_timer(60.0, self._publish_standby_heartbeat)
        self.get_logger().info(
            f'RecordDataNode started | model={model_path} '
            f'conf={self._confidence} class={self._target_class} '
            f'timeout={self._timeout_sec}s'
        )

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def _publish_camera_record_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self._camera_record_mode_pub.publish(msg)

    def _image_cb(self, msg: Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.yolo_model.predict(
            cv_image,
            imgsz=640,
            conf=self._confidence,
            device=self._yolo_device,
            verbose=False,
        )

        target_detected = False
        if results and hasattr(results[0], 'boxes'):
            for box in results[0].boxes:
                if int(box.cls[0]) == self._target_class:
                    target_detected = True
                    break

        if target_detected:
            self._trigger_pub.publish(Empty())
            self._start_or_extend_recording()

    def _start_or_extend_recording(self):
        now = time.time()

        if self._is_recording:
            self._recording_end = now + self._timeout_sec
            return

        self._is_recording  = True
        self._recording_end = now + self._timeout_sec
        self._publish_camera_record_mode('record')
        self.get_logger().info(
            f'YOLO class {self._target_class} detected: camera raw recording requested'
        )

    def _check_recording_status(self):
        self._publish_status('RECORDING' if self._is_recording else 'SCANNING')
        if not self._is_recording:
            return
        if time.time() > self._recording_end:
            self._is_recording   = False
            self._publish_camera_record_mode('standby')
            self.get_logger().info('Camera raw recording standby requested')

    def _publish_record_heartbeat(self):
        if self._is_recording:
            self._publish_camera_record_mode('record')

    def _publish_standby_heartbeat(self):
        if not self._is_recording:
            self._publish_camera_record_mode('standby')

    def destroy_node(self):
        if self._is_recording:
            self._publish_camera_record_mode('standby')
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
