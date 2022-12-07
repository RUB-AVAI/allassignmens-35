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


from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from PyQt5.QtWidgets import *

sys.path.append(os.path.dirname(__file__)+"/../camera_pkg")
from gui_node import MainWindow, GuiNode


@pytest.mark.rostest
def generate_test_description():
    file_path = os.path.dirname(__file__)
    gui_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(
            file_path, "..", "camera_pkg", "gui_node.py"
        )],
        additional_env={"PYTHONUNBUFFERED": "1"},
        parameters=[{
            # ros2 parameter
        }]
    )

    return (
        launch.LaunchDescription([
            gui_node,
            launch_testing.actions.ReadyToTest()
        ]),
        {
            "gui": gui_node
        }
    )


class TestGuiNode(unittest.TestCase):
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
        """app = QApplication(sys.argv)
        self.gui_node = GuiNode()
        self.hmi = MainWindow()

        self.hmi.node = self.gui_node
        self.gui_node.hmi = self.hmi"""

    def tearDown(self):
        self.node.destroy_node()

    def test_gui_set_frequency(self, gui, proc_output):
        """time.sleep(2)
        self.hmi = MainWindow.instance
        msgs_rx = []

        sub = self.node.create_subscription(
            Float64,
            "set_frequency",
            lambda msg: msgs_rx.append(msg),
            10
        )

        time.sleep(2)
        try:
            self.hmi.frequencyTxt.setText("2")

            self.hmi.set_recording_frequency()
            rclpy.spin_once(self.node, timeout_sec=0.1)
            self.assertGreater(len(msgs_rx), 0, "did no publish frequency")
            dummy_msg = Float64()
            dummy_msg.data = 2.0
            self.assertEqual(msgs_rx[0], dummy_msg, "published frequency is not correct")
        finally:
            self.node.destroy_subscription(sub)
        """