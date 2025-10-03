#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import time

from ultralytics import YOLO

detection_model = YOLO("yolo11n.pt")
#segmentation_model = YOLO("yolo11n-seg.pt")

# Export the model to NCNN format
detection_model.export(format="ncnn")
#segmentation_model.export(format="ncnn")

# Load the exported NCNN model
ncnn_detection_model = YOLO("yolo11n_ncnn_model")
#ncnn_segmentation_model = YOLO("yolo11n-seg_ncnn_model")

class CameraSubscriber(Node):

    global ncnn_detection_model

    def __init__(self):
        super().__init__("camera_subscriber")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image, "camera_image", self.image_callback, 2)

        self.publisher_detection = self.create_publisher(Image, "detection_image", 2)
        self.publisher_segmentation = self.create_publisher(Image, "segmentation_image", 2)

        self.timer = self.create_timer(0.1, self.publish_detection_image)
        self.timer_segmentation = self.create_timer(0.1, self.publish_segmentation_image)

        self.get_logger().info("Camera Subscriber Node has been started.")

    def publish_detection_image(self):
        # This method is intended to publish detection images
        pass

    def publish_segmentation_image(self):
        # This method is intended to publish segmentation images
        pass

    def image_callback(self, msg):
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = ncnn_detection_model(frame, conf=0.45, iou=0.45, device='cpu')

        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            scores = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()

            for box, score, cls in zip(boxes, scores, classes):
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{result.names[int(cls)]} {score:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()