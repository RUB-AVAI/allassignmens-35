import os
import sys
import time
import unittest

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
sys.path.append(os.path.dirname(__file__)+"/../camera_pkg")
from camera_node import CameraNode
from std_msgs.msg import Float64
from sensor_msgs.msg import Image


@pytest.mark.rostest
def generate_test_description():
    file_path = os.path.dirname(__file__)
    camera_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(
            file_path, "..", "camera_pkg", "camera_node.py"
        )],
        additional_env={"PYTHONUNBUFFERED": "1"},
        parameters=[{
            # ros2 parameter
        }]
    )

    return (
        launch.LaunchDescription([
            camera_node,
            launch_testing.actions.ReadyToTest()
        ]),
        {
            "camera": camera_node
        }
    )


class TestCameraNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node("test_camera_node")
        self.camera_node = CameraNode()

    def tearDown(self):
        self.node.destroy_node()
        self.camera_node.destroy_node()

    def test_try_and_publish_image(self, camera, proc_output):
        msgs_rx = []
        sub = self.node.create_subscription(
            Image,
            "raw_image",
            lambda msg: msgs_rx.append(msg),
            10
        )
        pub = self.node.create_publisher(
            Float64,
            "set_frequency",
            10
        )
        time.sleep(2)

        try:
            for i in range(1, 3+1):
                freq = Float64()
                freq.data = float(i)
                pub.publish(freq)

                end_time = time.time() + 10
                while time.time() < end_time:
                    rclpy.spin_once(self.camera_node, timeout_sec=0.1)
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                    if len(msgs_rx) > i*7:
                        break

                self.assertGreater(len(msgs_rx), i*7, f"camera did not publish images at frequency {i}")
                msgs_rx = []
        finally:
            self.node.destroy_subscription(sub)
            self.node.destroy_publisher(pub)

    def test_frequency_callback(self,camera,proc_output):
        msg = Float64()
        msg.data = 2.0
        self.camera_node.callback_set_frequency(msg)
        self.assertEqual(msg.data, self.camera_node.frequency_)

    def test_set_timer(self,camera,proc_output):
        msg = Float64()
        msg.data = 2.0
        self.camera_node.callback_set_frequency(msg)
        period = 1.0/self.camera_node.frequency_
        periodTimer = self.camera_node.publish_timer_.timer_period_ns
        self.assertEqual(period*1000000000,periodTimer/1.0)


