import time
import threading
import ctypes
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Import the String message type
from cv_bridge import CvBridge

from arena_api.system import system
from arena_api.buffer import *

TAB1 = "  "
TAB2 = "    "

class HardwareTriggerImagePublisher(Node):
    def __init__(self):
        super().__init__('hardware_trigger_image_publisher')
        self.publisher_ = self.create_publisher(Image, '/image', 10)
        # Create a heartbeat publisher for the camera_heartbeat topic.
        self.heartbeat_publisher = self.create_publisher(String, 'camera_heartbeat', 10)
        self.bridge = CvBridge()  # Initialize cv_bridge

        self.get_logger().info("Initializing device...")
        self.devices = self.create_devices_with_tries()
        self.device = system.select_device(self.devices)
        self.nodemap = self.device.nodemap
        self.nodes = self.get_nodes(self.nodemap)
        self.configure_trigger_acquire_image(self.device, self.nodes)

        self.get_logger().info(f'{TAB1}Starting image stream...')
        self.device.start_stream()

        # Start the acquisition thread.
        self._running = True
        self.acquisition_thread = threading.Thread(target=self.acquire_images)
        self.acquisition_thread.start()

    def create_devices_with_tries(self):
        tries = 0
        tries_max = 6
        sleep_time_secs = 10
        devices = None
        while tries < tries_max:
            devices = system.create_device()
            if not devices:
                self.get_logger().info(
                    f'{TAB1}Try {tries+1} of {tries_max}: waiting for {sleep_time_secs} secs for a device to be connected!'
                )
                for sec_count in range(sleep_time_secs):
                    time.sleep(1)
                    self.get_logger().info(f'{TAB1}{sec_count+1} seconds passed', throttle_duration_sec=1.0)
                tries += 1
            else:
                return devices
        raise Exception(f'{TAB1}No device found! Please connect a device and run the node again.')

    def get_nodes(self, nodemap):
        nodes = nodemap.get_node([
            'TriggerSelector', 'TriggerMode', 'TriggerSource',
            'LineSelector', 'LineMode', 'TriggerActivation', 'Width', 'Height', 'PixelFormat'
        ])
        return nodes

    def configure_trigger_acquire_image(self, device, nodes):
        nodes['Width'].value = 2880
        nodes['Height'].value = 1860
        nodes['PixelFormat'].value = 'RGB8'
        self.num_channels = 3
        nodes['LineSelector'].value = 'Line2'
        nodes['LineMode'].value = 'Input'
        self.get_logger().info(f'{TAB1}Set trigger selector to FrameStart')
        nodes['TriggerSelector'].value = 'FrameStart'
        self.get_logger().info(f'{TAB1}Enable trigger mode')
        nodes['TriggerMode'].value = 'On'
        nodes['TriggerActivation'].value = 'RisingEdge'
        nodes['TriggerSource'].value = 'Line2'
        self.get_logger().info(f'{TAB1}Set trigger source to Line1')
        tl_stream_nodemap = device.tl_stream_nodemap
        tl_stream_nodemap['StreamAutoNegotiatePacketSize'].value = True
        tl_stream_nodemap['StreamPacketResendEnable'].value = True
        self.get_logger().info(f'{TAB1}Trigger and stream configuration complete.')

    def form_image_msg(self, item):
        """
        Convert the raw image buffer (item.pbytes) into a NumPy array, then
        use cv_bridge to convert it into a sensor_msgs/Image message.
        Assumes the image is in rgb8 format (3 channels).
        """
        buffer_bytes_per_pixel = int(len(item.data) / (item.width * item.height))
        # Wrap the raw buffer pointer as a ctypes array.
        array = (ctypes.c_ubyte * (self.num_channels * item.width * item.height)).from_address(
            ctypes.addressof(item.pbytes)
        )
        # Create a NumPy array from the ctypes array.
        npndarray = np.ndarray(buffer=array, dtype=np.uint8, shape=(item.height, item.width, buffer_bytes_per_pixel))
        # Convert the NumPy array into a ROS Image message using cv_bridge.
        img_msg = self.bridge.cv2_to_imgmsg(npndarray, encoding="rgb8")
        # Set the timestamp and frame_id in the image message header.
        img_msg.header.stamp.sec = int(item.timestamp_ns / 1_000_000_000)
        img_msg.header.stamp.nanosec = int(item.timestamp_ns % 1_000_000_000)
        img_msg.header.frame_id = str(item.frame_id)
        return img_msg

    def acquire_images(self):
        while self._running and rclpy.ok():
            try:
                self.get_logger().debug(f'{TAB2}Waiting for buffer...')
                buffer = self.device.get_buffer()

                # Copy the buffer and requeue it to avoid running out of buffers.
                item = BufferFactory.copy(buffer)
                self.device.requeue_buffer(buffer)

                self.get_logger().debug(
                    f'{TAB2}Buffer received | [Width = {item.width} pxl, Height = {item.height} pxl]'
                )
                self.get_logger().debug(f'{TAB2}Processing image.')
                t0 = time.perf_counter()
                image_msg = self.form_image_msg(item)
                t1 = time.perf_counter()
                self.get_logger().debug(f'{TAB2}Image processing took {t1 - t0:.3f} seconds.')

                self.get_logger().debug(f'{TAB2}Publishing image.')
                self.publisher_.publish(image_msg)
                self.get_logger().debug(f'{TAB2}Published image.')

                # Publish a camera heartbeat message along with the image.
                heartbeat_msg = String()
                heartbeat_msg.data = (
                    f"Camera heartbeat: published image with frame id {image_msg.header.frame_id} "
                    f"at time {image_msg.header.stamp.sec}.{image_msg.header.stamp.nanosec}"
                )
                self.heartbeat_publisher.publish(heartbeat_msg)
                self.get_logger().debug(f'{TAB2}Published camera heartbeat.')

                # Destroy the copied item to prevent memory leaks.
                BufferFactory.destroy(item)

            except Exception as e:
                error_msg = f'Error acquiring image: {str(e)}'
                self.get_logger().error(f'Error acquiring image: {str(e)}')
                heartbeat_msg = String()
                heartbeat_msg.data = error_msg
                self.heartbeat_publisher.publish(heartbeat_msg)

        self.get_logger().info(f'{TAB1}Stopping stream...')
        self.device.stop_stream()
        system.destroy_device()
        self.get_logger().info(f'{TAB1}Destroyed all created devices.')

    def destroy_node(self):
        self._running = False
        if self.acquisition_thread.is_alive():
            self.acquisition_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareTriggerImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()