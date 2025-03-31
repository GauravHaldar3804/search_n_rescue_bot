#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class PiCameraNode(Node):
    def __init__(self):
        super().__init__('pi_camera_node')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.capture_image)  # Publish at ~10 FPS
        self.bridge = CvBridge()

        # Initialize the camera using V4L2 backend
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        # Lower the resolution and frame rate to reduce memory usage
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Debug: Log camera properties
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Camera properties: width={width}, height={height}, FPS={fps}")

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open Raspberry Pi Camera Module")
        else:
            self.get_logger().info("Camera node started, publishing images")

    def capture_image(self):
        try:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to capture image: ret is False")
                return

            if frame is None:
                self.get_logger().warn("Captured frame is None")
                return

            if frame.size == 0:
                self.get_logger().warn("Captured frame is empty (size 0)")
                return

            # Convert the captured frame to a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(ros_image)
            self.get_logger().info("Published image frame")
        except Exception as e:
            self.get_logger().error(f"Exception in capture_image: {e}")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PiCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
