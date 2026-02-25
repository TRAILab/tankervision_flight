import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge # To convert ROS Image messages to OpenCV
from ultralytics import YOLO
import time
from datetime import datetime
from .file_upload import start_upload_service
# Get current datetime
import os
import subprocess

class AllTopicsBagWriterNode(Node):
    def __init__(self):
        super().__init__('all_topics_bag_writer_node')

        # Format it to string: YYYY-MM-DD_HH-MM

        # Load YOLO model (e.g., YOLOv8 nano for speed)
        self.yolo_model = YOLO('yolo11n-seg.pt') # Download pretrained model or use your own
        self.cv_bridge = CvBridge()
        self.path_root = '/media/trail/Wildfire1/Downloads'
        # Recording state
        self.is_recording = False
        self.recording_end_time = 0
        self.subscription = self.create_subscription(
            Image,              # Message type
            '/cam0/image_raw',
            self.image_callback,  # Callback function
            3                   # QoS profile (queue size)
        )
        # Timer to periodically check for new topics

        # Timer to manage recording duration
        self.recording_timer = self.create_timer(1, self.check_recording_status)
        self.process = None
        self.get_logger().info('Started node with YOLO detection, recording to: ' + self.bag_path)


    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        # Run YOLO detection
        results = self.yolo_model(cv_image, conf=0.5)
        # Check if any objects are detected
        if len(results[0].boxes) > 0: # If detections exist
            self.start_recording()
            self.get_logger().info(f'Detection triggered recording: {len(results[0].boxes)} objects')

        # Record the image if in recording mode
        if self.is_recording:
            self.generic_callback('/cam0/image_raw', msg)



    def start_recording(self):
        now = datetime.now()

        timestamp_str = now.strftime("%Y-%m-%d_%H-%M")

        self.bag_path = os.path.join(self.path_root, '{timestamp_str}.bag')
        self.is_recording = True
        self.process = subprocess.Popen(["ros2", "bag", "record",  "-a", "-b", "256", "-o", self.bag_path, ])
        self.recording_end_time = time.time() + 60 # Record for 1 minute (60 seconds)
        self.get_logger().info('Started recording for 1 minute due to detection')

    def check_recording_status(self):
        if self.is_recording and time.time() > self.recording_end_time:
            self.is_recording = False
            workers, monitor_thread = start_upload_service()
            self.get_logger().info('Stopped recording after 1 minute')
            while True:
               if not os.listdir(self.path_root):
                   break
            for w in workers:
                w.join()
            monitor_thread.join()
            if self.process is not None:
                self.process.terminate()
            print("Reviving node")



    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AllTopicsBagWriterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

