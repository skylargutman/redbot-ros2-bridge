#!/usr/bin/env python3
"""
Camera Node for RedBot
Publishes images from Raspberry Pi Camera using picamera2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from picamera2 import Picamera2
import numpy as np
import os
import time


class CameraNode(Node):
    """Publishes camera images to ROS2"""

    def __init__(self):
        super().__init__('camera_node')

        # Parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('framerate', 30)

        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        framerate = self.get_parameter('framerate').value

        # Publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Initialize camera
        self.get_logger().info(f'Initializing camera at {width}x{height} @ {framerate}fps')

        os.environ['LIBCAMERA_LOG_LEVELS'] = '*:ERROR'
        self.picam2 = Picamera2()

        config = self.picam2.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.picam2.configure(config)

        # Don't use display preview
        time.sleep(1)

        self.picam2.start()

        # Timer to capture and publish
        timer_period = 1.0 / framerate
        self.timer = self.create_timer(timer_period, self.capture_and_publish)

        self.get_logger().info('Camera node started')

    def capture_and_publish(self):
        """Capture frame and publish to ROS2"""
        try:
            # Capture frame
            frame = self.picam2.capture_array()

            # Convert to ROS2 Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_link"

            # Publish
            self.image_pub.publish(img_msg)

            # Publish camera info (basic)
            camera_info_msg = CameraInfo()
            camera_info_msg.header = img_msg.header
            camera_info_msg.height = frame.shape[0]
            camera_info_msg.width = frame.shape[1]
            self.camera_info_pub.publish(camera_info_msg)

        except Exception as e:
            self.get_logger().error(f'Error capturing frame: {e}')

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.picam2.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down camera node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
