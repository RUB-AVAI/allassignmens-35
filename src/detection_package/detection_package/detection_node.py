#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolov5.models import common
import cv_bridge


class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection")
        self.subscriber_ = self.create_subscription(Image, "raw_image", self.callback_raw_image, 10)
        self.publisher_ = self.create_publisher(Image, "detected_image", 10)
        self.weights = 'yolov5s-int8_edgetpu.tflite' #Placeholder
        self.data = 'labels.yaml' # Placeholder
        self.device = 'cpu'
        self.dnn = False
        self.half = False
        self.cv_bridge_ = cv_bridge.CvBridge()
        self.model = common.DetectMultiBackend(weights=self.weights, device=self.device, dnn=self.dnn, data=self.data, fp16=self.half)
        self.stride = self.model.stride
        self.names = self.model.names
        self.pt = self.model.pt

    def callback_raw_image(self, msg):
        self.get_logger().info("Image to detect received")
        raw_image = self.cv_bridge_.imgmsg_to_cv2(msg)
        pred = self.model(raw_image)
        pred = self.cv_bridge_.cv2_to_imgmsg(pred.render())
        self.publisher_.publish(pred)
        self.get_logger().info("Published detected image.")


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()