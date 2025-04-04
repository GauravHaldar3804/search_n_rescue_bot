#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2, Picamera2Exception
import cv2
import numpy as np
import time


class PiCam2Node(Node):
    def __init__(self):
        super().__init__('picam2_node')

        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(
                main={"format": 'BGR888', "size": (640, 480)}
            )
            self.picam2.configure(config)
            self.picam2.start()
            self.get_logger().info("Picamera2 started successfully")
        except Picamera2Exception as e:
            self.get_logger().error(f"Failed to initialize Picamera2: {e}")
            rclpy.shutdown()
            return
        except Exception as e:
            self.get_logger().error(f"Unexpected error initializing Picamera2: {e}")
            rclpy.shutdown()
            return

        # Timer to capture images
        self.timer = self.create_timer(0.1, self.capture_and_publish)

    def capture_and_publish(self):
        try:
            frame = self.picam2.capture_array()
            if frame is None or frame.size == 0:
                self.get_logger().warn("Empty frame captured")
                return

            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info("Published image frame")

        except Exception as e:
            self.get_logger().error(f"Error capturing or publishing frame: {e}")

    def destroy_node(self):
        try:
            self.picam2.stop()
        except Exception as e:
            self.get_logger().warn(f"Error stopping camera: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PiCam2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node due to keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
