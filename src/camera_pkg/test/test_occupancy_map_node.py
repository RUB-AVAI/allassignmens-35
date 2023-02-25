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
from occupancy_map_node import OccupancyMapNode
from sensor_msgs.msg import Image, CompressedImage


#launch feature node
@pytest.mark.rostest
def generate_test_description():
    file_path = os.path.dirname(__file__)
    occupancy_map_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..", "camera_pkg", 'occupancy_map_node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        parameters=[{
        }]
    )
    return (
        launch.LaunchDescription([
            occupancy_map_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'occupancy_map': occupancy_map_node,
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
        self.node = rclpy.create_node("test_occupancy_map_node")
        self.occupancy_map_node = OccupancyMapNode()

    def tearDown(self):
        self.node.destroy_node()
        self.occupancy_map_node.destroy_node()

    def test_lidar_to_xy(self, occupancy_map, proc_output):
        result = self.occupancy_map_node.lidar_to_xy((31.25,10.0,2))
        self.assertEqual(result, (10+self.occupancy_map_node.turtle_state["x"], self.occupancy_map_node.turtle_state["y"], 2), "lidar to xy wrong result")
        result = self.occupancy_map_node.lidar_to_xy((45.0+31.25,0.0,0))
        self.assertEqual(result, (self.occupancy_map_node.turtle_state["x"],self.occupancy_map_node.turtle_state["y"],0), "lidar to xy wrong result")

    def test_clustering(self, occupancy_map, proc_output):
        lidar_test_points = [(31.25,10.0,0),(5.0,4.0,1),(5.0,4.0,1),(-20.2,10.6,2),(-10.2,9.6,2),(-18.8,9.5,2),(31.25,11.0,0)]
        self.occupancy_map_node.update_map(lidar_test_points)
        self.assertEqual(self.occupancy_map_node.map,
                         [(15.0, 5.0, 0), (8.587490966130753, 3.230845239123995, 1),
                          (8.587490966130753, 3.230845239123995, 1),
                          (11.605891952076172, -3.289884891691516, 2),
                          (12.195523333865431, -1.3548756047461774, 2),
                          (11.100129218721444, -2.282748349002656, 2), (16.0, 5.0, 0)],
                         "Clustering test failed.")