#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge
import math
from typing import Tuple, List

MAP_SIZE = 251  # map is 50.2m x 50.2m


class OccupancyMapNode(Node):
    def __init__(self):
        super().__init__("occupancy_map")

        # one cell is 20cm x 20cm
        # map[x][y]
        self.map = [[-1] * MAP_SIZE for x in range(MAP_SIZE)]
        # start position in self.map and angle in radians
        self.turtle_state = {"x": 125, "y": 125, "angle": math.radians(0)}

        # to do: to add correct message type
        self.publisher_ = self.create_publisher(Float64, "updated_map", 10)
        # to do: add subscriber for turtlebot movement

        """ Debug Tests
        # [Angle, Distance, ClassID]
        test_positions = [(-90, 30, 0), (60, 44, 1), (0, 21, 2), (45, 67, 2)] #data from sensor fusion
        # (125,95), (147,163.105), (146, 125), (172.376, 172.376)
        tmp = []
        for i in test_positions:
            a = list(i)
            a[0] = math.radians(a[0])
            tmp.append(tuple(a))
        test_positions = tmp

        self.update_map(test_positions)
        
        for x in range(MAP_SIZE):
            for y in range(MAP_SIZE):
                if self.map[x][y] != -1:
                    self.get_logger().info(f"{x},{y}, class={self.map[x][y]}")
        """
        self.get_logger().info("Occupancy Map started.")

    def lidar_to_xy(self, data: Tuple[float,float,int]):
        """
        Converts a lidar point (relative to the turtlebot) to xy coordinates on the map.
        :param data: Tuple in the format (angle, distance, classID).
        :return: Dictionary in the format (x-coordinate, y-coordinate, classID). All coordiantes are real world
        coordinates in cm.
        """
        x = math.cos(data[0] + self.turtle_state["angle"])*data[1] + self.turtle_state["x"]
        y = math.sin(data[0] + self.turtle_state["angle"])*data[1] + self.turtle_state["y"]
        return {"x": x, "y": y, "classID": data[2]}

    def update_map(self, positions: List[Tuple[float, float, int]]):
        """
        Places the classes of all lidar point in a given list into the map.
        :param positions: A list of tuples, representing the lidar points in the format (angle, distance, classID).
        """
        #positions list[tuple[float, float, int]]
        for position in positions:
            xyc = self.lidar_to_xy(position)
            self.map[math.floor(xyc["x"]/2)][math.floor(xyc["y"]/2)] = xyc["classID"]
        self.publisher_.publish(self.map)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
