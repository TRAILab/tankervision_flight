import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import datetime

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        # Subscribe to the Image topic (adjust the topic name as needed)
        self.subscription = self.create_subscription(
            Image,
            '/cam0/image_raw',  # Change this to your image topic name
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.base_dir = '/media/trail/Wildfire1/data/images'  # Base directory for storing images

    def listener_callback(self, msg: Image):
        try:
            # Convert the ROS Image message to an OpenCV image (BGR8 encoding assumed)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # Extract timestamp from the message header.
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec

        # Compute the total nanoseconds
        total_nsec = sec * 1_000_000_000 + nsec

        # Get a date string for the directory (e.g., 2025-04-02)
        dt = datetime.datetime.fromtimestamp(sec + nsec * 1e-9)
        date_str = dt.strftime("%Y-%m-%d")

        # Create the directory if it doesn't exist.
        dir_path = os.path.join(self.base_dir, date_str)
        os.makedirs(dir_path, exist_ok=True)

        # Construct the file path using the total nanoseconds as the filename.
        file_path = os.path.join(dir_path, f"{total_nsec}.png")

        # Save the image as a PNG file.
        success = cv2.imwrite(file_path, cv_image)
        if not success:
            self.get_logger().error(f"Failed to save image to {file_path}")
        else:
            self.get_logger().info(f"Saved image: {file_path}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()