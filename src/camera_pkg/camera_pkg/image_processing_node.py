#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pandas
import sys
import os
from sensor_msgs.msg import Image, CompressedImage
import cv2
import yolov5.models.common
from std_msgs.msg import Float64MultiArray
import numpy as np
from avai_messages.msg import FloatArray
from avai_messages.msg import FloatList

from cv_bridge import CvBridge

#MODEL_PATH = "/home/ubuntu/allassingmens-35/src/camera_pkg/camera_pkg/best-int8_edgetpu.tflite"
MODEL_PATH = "/home/ubuntu/allassignmens-35/src/camera_pkg/camera_pkg/best.pt"
LABEL_PATH = "/home/ubuntu/allassingmens-35/src/camera_pkg/camera_pkg/labels.yaml"


class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__("image_processor")

        self.interpreter = yolov5.models.common.DetectMultiBackend(MODEL_PATH, data=LABEL_PATH)
        self.interpreter = yolov5.models.common.AutoShape(self.interpreter)

        self.cv_bridge_ = CvBridge()

        self.publisher_boundingBoxes = self.create_publisher(FloatArray,"bounding_box",10)
        self.subscriber_ = self.create_subscription(Image, "raw_image", self.callback_raw_image, 10)
        self.publisher_ = self.create_publisher(CompressedImage, "processed_image", 10)
        self.get_logger().info("Image Processor started.")

    def callback_raw_image(self, msg):
        self.get_logger().info("Received raw image.")
        raw_image = self.cv_bridge_.imgmsg_to_cv2(msg)

        raw_image = cv2.cvtColor(raw_image, cv2.COLOR_RGB2BGR)
        detect = self.interpreter(raw_image)
        detect.render()

        boxes = detect.pandas().xywhn[0].to_numpy().astype(np.float)

        bounding_Boxes = []
        for i in range(0,len(boxes)):
            new_box = []
            for j in range(0,len(boxes[0])):
                new_box.append(boxes[i][j])

            bounding_Boxes.append(new_box)

        float_array = FloatArray()

        for p in range(0,len(bounding_Boxes)):
            msgBoxes = FloatList()
            msgBoxes.elements = bounding_Boxes[p]
            float_array.lists.append(msgBoxes)
            #float_array.lists[i] = msgBoxes




        self.publisher_boundingBoxes.publish(float_array)
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
