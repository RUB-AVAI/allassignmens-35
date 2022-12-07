import os
import sys
import time
import unittest
import cv_bridge
import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage


#launch feature node
@pytest.mark.rostest
def generate_test_description():
    file_path = os.path.dirname(__file__)
    image_processing_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..", 'image_processing_node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        parameters=[{
        }]
    )
    return (
        launch.LaunchDescription([
            image_processing_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'image_processing': image_processing_node,
        }
    )

class TestImageProcessingLink(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_image_processing_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_image_processing_transmits(self, image_processing, proc_output):
        msgs_rx = []

        pub = self.node.create_publisher(Image, "raw_image", 10)
        sub = self.node.create_subscription(
            CompressedImage,
            "processed_image",
            lambda msg: msgs_rx.append(msg),
            10
        )

        try:
            msg = Image()
            raw_image = cv2.imread("/home/ubuntu/allassignmens-35-main/src/camera_pkg/camera_pkg/test/ManualImage25.png")
            cv_bridge_ = CvBridge()
            msg = cv_bridge_.cv2_to_imgmsg(raw_image)
            time.sleep(5)
            pub.publish(msg)
            #Wait until the talker transmits two messages over the ROS topic
            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if len(msgs_rx) > 2:
                    break

            self.assertEqual(len(msgs_rx), 1)
        finally:
            self.node.destroy_subscription(sub)
            self.node.destroy_publisher(pub)
    '''
    def test_image_processing_receives(self, image_processing, proc_output):
        pub = self.node.create_publisher(
            std_msgs.msg.String,
            'raw_image',
            10
        )
        try:
            msg = std_msgs.msg.String()
            msg.data = str(uuid.uuid4())
            for _ in range(10):
                pub.publish(msg)
                success = proc_output.waitFor(
                    expected_output=msg.data,
                    process=image_processing,
                    timeout=1.0,
                )
                if success:
                    break
            assert success, 'Waiting for output timed out'
        finally:
            self.node.destroy_publisher(pub)
    '''