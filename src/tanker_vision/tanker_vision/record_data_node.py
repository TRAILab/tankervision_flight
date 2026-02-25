import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import time
from datetime import datetime
import os
import subprocess
from std_msgs.msg import String


class RecordDataNode(Node):
    def __init__(self):
        super().__init__('record_data_node')
        
        # Load YOLO model (assuming a YOLOv8 variant with segmentation if needed)
        self.yolo_model = YOLO('/home/trail/Desktop/TANKER_VISION/ros2_ws/src/tanker_vision/tanker_vision/fire_detection.pt', verbose=False)
        self.cv_bridge = CvBridge()
        self.path_root = '/mnt/wildfire/data'  # Path to save the bag files
        self.record_since_last_detection = 20  # seconds to record after last detection
        
        # Recording state variables
        self.is_recording = False
        self.recording_end_time = 0
        self.process = None
        self.image_count = 0

        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/cam0/image_raw',
            self.image_callback,
            3  # QoS profile (queue size)
        )
        
        # Timer to manage recording duration
        self.recording_timer = self.create_timer(1, self.check_recording_status)

        self.status_publisher = self.create_publisher(String, '/record_data/status', 10)
        self.get_logger().info("Record Data Node started.")

    def publish_status(self, status_str):
        msg = String()
        msg.data = status_str
        self.status_publisher.publish(msg)

    def image_callback(self, msg):
        self.image_count += 1
        # Convert ROS Image message to an OpenCV image (RGB8 encoding)
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run YOLO inference on the image
        results = self.yolo_model(cv_image, conf=0.3, verbose=False) # show=True is not needed for processing, remove it if not required
        
        # Check if the class of interest is detected
        detected_class = False
        # Assuming results[0].boxes is iterable and each box has a .cls attribute,
        # and that self.yolo_model.names is a mapping from class indices to names.
        if results and hasattr(results[0], "boxes"):
            for box in results[0].boxes:
                if int(box.cls) == 2:
                    detected_class = True
                    break

        # If a car is detected, start recording or reset the timer if already recording
        if detected_class:  # Check every 10 images
            self.get_logger().info("Fire detected, starting or resetting recording timer.")
            self.start_or_reset_recording()
        
        if not self.is_recording:
            self.publish_status("SCANNING")
        elif self.process and self.process.poll() is None:
            self.publish_status("RECORDING")


    def  start_or_reset_recording(self):
        if self.is_recording:
            self.recording_end_time = time.time() + self.record_since_last_detection
        else:
            now = datetime.now()
            ts = now.strftime("%Y-%m-%d_%H-%M-%S")
            self.bag_path = os.path.join(self.path_root, ts)
            cmd = [
                "/opt/ros/humble/bin/ros2", "bag", "record",
                "-o", self.bag_path,
                "-a", "--compression-mode", "message",
                "--compression-format", "zstd", "-b", "100000000"
            ]
            self.process = subprocess.Popen(cmd)
            self.is_recording = True
            self.recording_end_time = time.time() + self.record_since_last_detection
            self.get_logger().info(f"Started bag recording to {self.bag_path}")
            
    def check_recording_status(self):
        # Check if the recording duration has elapsed
        if self.is_recording and time.time() > self.recording_end_time:
            # Terminate the bag recording subprocess if it is still running
            if self.process is not None:
                self.process.terminate()
                self.is_recording = False
            self.get_logger().info("Stopped bag recording after {} seconds.".format(self.record_since_last_detection))

    def destroy_node(self):
        super().destroy_node()
        if self.process is not None:
            self.process.terminate()

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