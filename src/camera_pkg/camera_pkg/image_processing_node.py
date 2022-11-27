#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
import yolov5.models.common

from cv_bridge import CvBridge

#MODEL_PATH = "./yolov5s-int8_edgetpu.tflite"
MODEL_PATH = "./best.pt"
LABEL_PATH = "./labels.yaml"


class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__("image_processor")

        self.interpreter = yolov5.models.common.DetectMultiBackend(MODEL_PATH, data=LABEL_PATH)
        self.interpreter = yolov5.models.common.AutoShape(self.interpreter)

        self.cv_bridge_ = CvBridge()

        self.subscriber_ = self.create_subscription(Image, "raw_image", self.callback_raw_image, 10)
        self.publisher_ = self.create_publisher(CompressedImage, "processed_image", 10)
        self.get_logger().info("Image Processor started.")

    def callback_raw_image(self, msg):
        self.get_logger().info("Received raw image.")
        raw_image = self.cv_bridge_.imgmsg_to_cv2(msg)

        raw_image = cv2.cvtColor(raw_image, cv2.COLOR_RGB2BGR)
        detect = self.interpreter(raw_image)
        detect.render()
        processed_image = cv2.cvtColor(raw_image, cv2.COLOR_RGB2BGR)
        # Process image

        processed_image = self.cv_bridge_.cv2_to_compressed_imgmsg(processed_image, "png")
        self.publisher_.publish(processed_image)
        self.get_logger().info("Published processed image.")


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
