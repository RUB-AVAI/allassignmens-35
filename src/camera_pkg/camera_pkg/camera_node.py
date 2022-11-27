#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera")

        self.frequency_ = 1

        self.cap_ = cv2.VideoCapture(0)#"rtsp://web.nidaku.de:8554/
        self.cap_ = cv2.imread("./ManualImage25.png")
       # self.cap_.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        #self.cap_.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cv_bridge_ = CvBridge()

        self.subscriber_ = self.create_subscription(Float64, "set_frequency", self.callback_set_frequency, 10)

        self.publisher_ = self.create_publisher(Image, "raw_image", 10)
        self.publish_timer_ = None

        self.get_logger().info("Camera started.")

    def __del__(self):
        self.cap_.release()

    def callback_set_frequency(self, msg):
        self.get_logger().info("Received new frequency")
        self.frequency_ = msg.data
        self.set_timer()

    def set_timer(self):
        if self.publish_timer_ is not None:
            self.destroy_timer(self.publish_timer_)
            self.get_logger().info("Stopped timer")
        if self.frequency_ > 0:
            self.publish_timer_ = self.create_timer(1.0/self.frequency_, self.try_and_publish_image)
            self.get_logger().info("Set timer to frequency " + str(self.frequency_))

    def try_and_publish_image(self):
        #ret, frame = self.cap_.read()
       # if ret:
            self.publisher_.publish(self.cv_bridge_.cv2_to_imgmsg(self.cap_))
            self.get_logger().info("Published raw image.")
       # else:
            self.get_logger().info("Could not get raw image.")


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
