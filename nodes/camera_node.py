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
       self.timer = self.create_timer(0.1, self.capture_image)  # Publish at 10 FPS
       self.bridge = CvBridge()
      
       # Initialize camera
       self.cap = cv2.VideoCapture(0)
       self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
       self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
       self.cap.set(cv2.CAP_PROP_FPS, 30)
      
       if not self.cap.isOpened():
           self.get_logger().error("Failed to open Raspberry Pi Camera Module")
       else:
           self.get_logger().info("Camera node started, publishing images")


   def capture_image(self):
       ret, frame = self.cap.read()
       if ret:
           ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
           self.publisher.publish(ros_image)
           self.get_logger().info("Published image frame")
       else:
           self.get_logger().warn("Failed to capture image")


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





