#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from avai_messages.msg import PointArray, ClassedPoint, FloatArray, FloatList
import math
from typing import Tuple, List

MAP_SIZE = 25  # map is 5m x 5m


class OccupancyMapNode(Node):
    def __init__(self):
        super().__init__("occupancy_map")

        # one cell is 20cm x 20cm
        # map[x][y]
        self.map = [[-1] * MAP_SIZE for x in range(MAP_SIZE)]
        # start position (meter) in self.map and angle in radians
        self.turtle_state = {"x": float(MAP_SIZE*20/2/100), "y": float(MAP_SIZE*20/2/100), "angle": math.radians(0)}
        self.get_logger().info(str(self.turtle_state["x"]))

        self.publisher_ = self.create_publisher(PointArray, "updated_points", 10)
        self.subscriber_Lidar = self.create_subscription(FloatArray, "lidar_values", self.callback_lidar_values, 10)
        # to do: add subscriber for turtlebot movement

        self.get_logger().info("Occupancy Map started.")

    def callback_lidar_values(self, msg):
        lidar_values = []
        for lst in msg.lists:
            lidar = []
            for e in lst.elements:
                lidar.append(e)

            lidar_values.append(tuple(lidar))

        self.update_map(lidar_values)

    def lidar_to_xy(self, data: Tuple[float, float, int]):
        """
        Converts a lidar point (relative to the turtlebot) to xy coordinates on the map.
        :param data: Tuple in the format (angle, distance, classID). Angles are assumed counter-clockwise.
        Distance is given in meters
        :return: Dictionary in the format (x-coordinate, y-coordinate, classID). All coordiantes are real world
        coordinates in cm.
        """
        x = math.cos(-math.radians(data[0] + -self.turtle_state["angle"]))*data[1] + self.turtle_state["x"]
        y = math.sin(-math.radians(data[0] + -self.turtle_state["angle"]))*data[1] + self.turtle_state["y"]
        return {"x": x, "y": y, "classID": data[2]}

    def update_map(self, positions: List[Tuple[float, float, int]]):
        """
        Places the classes of all lidar point in a given list into the map.
        :param positions: A list of tuples, representing the lidar points in the format (angle, distance, classID).
        """
        #positions list[tuple[float, float, int]]

        points = []
        for position in positions:
            xyc = self.lidar_to_xy(position)

            p = ClassedPoint()
            p.x = xyc["x"]
            p.y = xyc["y"]
            p.c = int(xyc["classID"])

            points.append(p)
            self.map[math.floor(xyc["x"])][math.floor(xyc["y"])] = xyc["classID"]

        turtle_point = ClassedPoint()
        turtle_point.x = self.turtle_state["x"]
        turtle_point.y = self.turtle_state["y"]
        turtle_point.c = 3
        points.append(turtle_point)

        pointArray = PointArray()
        pointArray.data = points

        self.publisher_.publish(pointArray)
        self.get_logger().info("Published points list")


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
