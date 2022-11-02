#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge


class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__("image_processor")

        self.cv_bridge_ = CvBridge()

        self.subscriber_ = self.create_subscription(Image, "raw_image", self.callback_raw_image, 10)
        self.publisher_ = self.create_publisher(Image, "processed_image", 10)
        self.get_logger().info("Image Processor started.")

    def callback_raw_image(self, msg):
        self.get_logger().info("Received raw image.")
        raw_image = self.cv_bridge_.imgmsg_to_cv2(msg)

        # Process image
        processed_image = self.cv_bridge_.cv2_to_imgmsg(raw_image)

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
