#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        self.bridge = CvBridge()
        self.camera = cv2.VideoCapture(0)

        self.publisher_ = self.create_publisher(Image, "camera_image", 2)
        self.timer = self.create_timer(0.02, self.publish_image)

        self.get_logger().info("Camera Publisher Node has been started.")

    def publish_image(self):
        success, frame = self.camera.read()

        frame = cv2.resize(frame, (640, 640), interpolation=cv2.INTER_CUBIC)    # what this for?

        if success:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image_msg)

    def destroy_node(self):
        self.camera.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()