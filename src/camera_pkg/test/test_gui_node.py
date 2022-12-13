import asyncio
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
from random import Random

from avai_messages.msg import FloatArray, FloatList
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage

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
        self.node = rclpy.create_node("test_gui_node")
        self.gui_node = GuiNode()

    def tearDown(self):
        self.node.destroy_node()
        self.gui_node.destroy_node()

    def test_callback_process_boundingbox(self, gui, proc_output):
        #msgs_rx = []
        rand = Random(177013)

        pub = self.node.create_publisher(
            FloatArray,
            "bounding_box",
            10
        )
        time.sleep(3)
        try:
            for i in range(0, 3+1):
                #create test messages bith random bounding boxes
                float_array = FloatArray()

                test_bounding_boxes = []
                for j in range(i):
                    bbox = []
                    for v in range(7):
                        bbox.append(rand.random())
                    test_bounding_boxes.append(bbox)

                    float_list = FloatList()
                    float_list.elements = bbox
                    float_array.lists.append(float_list)

                #send and receive message
                pub.publish(float_array)

                end_time = time.time() + 5
                while time.time() < end_time:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                    rclpy.spin_once(self.gui_node, timeout_sec=0.1)
                    if i <= 0:
                        continue
                    if len(self.gui_node.bounding_boxes) >= i:
                        break

                self.assertEqual(self.gui_node.bounding_boxes, test_bounding_boxes, f"The {i} bounding boxrs were not processed correctly.")
        finally:
            self.node.destroy_publisher(pub)
