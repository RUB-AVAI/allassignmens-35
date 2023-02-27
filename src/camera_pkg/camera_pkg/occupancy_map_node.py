#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from avai_messages.msg import PointArray, ClassedPoint, FloatArray, FloatList, Point, Polygon, Polylines, TurtlebotState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
import math
from typing import Tuple, List
from sklearn.cluster import DBSCAN
import numpy as np
import message_filters
import tf_transformations
from math import atan2,pi

MAP_SIZE = 10  # map is assumed 10m x 10m
CAMERA_RANGE = 62.5  # camera field of view in degrees

#NEED TO CHECK BOUNDS? Maybe replace array with list

class OccupancyMapNode(Node):
    def __init__(self):
        super().__init__("occupancy_map")

        # one cell is 20cm x 20cm
        #map is a list of points (x,y,classID)
        self.map = []  # [[-1] * MAP_SIZE for x in range(MAP_SIZE)] # map[x][y]
        # start position (meter) in self.map and angle in degrees
        self.turtle_state = {"x": float(MAP_SIZE/2), "y": float(MAP_SIZE/2), "angle": 0}
        self.turtle_state_is_set = False
        self.middlepoints = []

        self.subscriber_middlepoint = self.create_subscription(Float64MultiArray, "target_point", self.get_middlepoint,
                                                               10)
        self.publisher_pointarray = self.create_publisher(PointArray, "updated_points", 10)
        self.publisher_polylines = self.create_publisher(Polylines, "polylines", 10)
        self.subscriber_lidar = message_filters.Subscriber(self, FloatArray, "lidar_values")
        self.subscriber_middlepoint = self.create_subscription(Float64MultiArray, "target_point", self.get_middlepoint, 10)
        self.subscriber_pose = message_filters.Subscriber(self, TransformStamped, "/turtlebot_pose")
        self.ts = message_filters.ApproximateTimeSynchronizer([self.subscriber_lidar, self.subscriber_pose], 100, .2)
        self.ts.registerCallback(self.callback_synchronised)

        self.get_logger().info("Occupancy Map started.")

    def callback_synchronised(self, msg_lidar, msg_odom):
        self.callback_pose(msg_odom)
        self.callback_lidar_values(msg_lidar)
        self.publish_map()

    def callback_pose(self, msg):
        # convert quaternion to euler angle
        """qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w"""
        qx = msg.transform.rotation.x
        qy = msg.transform.rotation.y
        qz = msg.transform.rotation.z
        qw = msg.transform.rotation.w
        r,p,y = \
            tf_transformations.euler_from_quaternion\
                ([qw, qx, qy, qz])
        #self.get_logger().info(f"position y{y}, p{p}, r{r}")
        #update turtle_state
        self.turtle_state["angle"] = math.degrees(-r)  # was (-3.14, 3.14), now counter clockwise 360
        #self.get_logger().info(f"{self.turtle_state['angle']}")
        self.turtle_state["x"] = float(MAP_SIZE/2) - msg.transform.translation.x#msg.pose.pose.position.x
        self.turtle_state["y"] = float(MAP_SIZE/2) - msg.transform.translation.y#msg.pose.pose.position.y
        self.turtle_state_is_set = True

    def callback_lidar_values(self, msg):
        if not self.turtle_state_is_set:
            return

        #self.get_logger().info("stamp "+str(msg.header.stamp.sec))

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
        :param data: Tuple in the format (angle, distance, classID). Angles are assumed counter-clockwise in degrees
        from 0 to CAMERA_RANGE-1. Distance is given in meters
        :return: List in the format [x-coordinate, y-coordinate, classID]. All coordinates are real world
        coordinates in meters.
        """
        absolute_angle = math.radians(data[0] + self.turtle_state["angle"]-CAMERA_RANGE/2)
        x = math.cos(absolute_angle)*data[1] + self.turtle_state["x"]
        y = math.sin(absolute_angle)*data[1] + self.turtle_state["y"]
        #self.get_logger().info(f"lidar: {data[0]} {data[1]} {data[2]}")
        return (x,y,data[2])  # {"x": x, "y": y, "classID": data[2]}

    def get_middlepoint(self, msg):
        middlepoint_map = msg.data
        # self.update_map(tuple(middlepoint_map))
        self.middlepoints.append(middlepoint_map)

    def update_map(self, positions: List[Tuple[float, float, int]]):
        """
        Places the classes of all lidar point in a given list into the map.
        :param positions: A list of tuples, representing the lidar points in the format (angle, distance, classID).
        """
        #for position in positions:
            #xyc = self.lidar_to_xy(position)
            #self.get_logger().info(f"xyc: {xyc}")

           # self.map[math.floor(xyc["x"]*100/20)][math.floor(xyc["y"]*100/20)] = xyc["classID"]
        if len(positions) <= 0:
            return

        # remove wrongly detected yellow cones, which probably should have been blue
        points_to_filter = []
        new_positions = [self.lidar_to_xy(new_lidar) for new_lidar in positions]
        for new_pos in new_positions:
            for point in self.map:
                if (point[2] == 0 and new_pos[2] == 2) or (point[2] == 2 and new_pos[2] == 0): # check only different colors
                    distance = math.sqrt((new_pos[0]-point[0])**2+(new_pos[1]-point[1])**2)
                    if distance < 0.2:
                        points_to_filter.append(new_pos)
                        points_to_filter.append(point)
                        #self.get_logger().info(f"new:{new_pos}, blue:{point}")
                        break
        #if not points_to_filter == []:
        #    self.get_logger().info(f"ptf:{points_to_filter}\nmap:{self.map}")
        points = self.map
        points += new_positions
        #points = [p for p in points if p not in points_to_filter]
        for point in points_to_filter:
            if point in points:
                points.remove(point)

        dbscan = DBSCAN(eps=0.13, min_samples=3)
        labels = dbscan.fit_predict(points)

        clusters = {}
        for i, label in enumerate(labels):
            if label != -1:

                if label not in clusters:
                    clusters[label] = []

                clusters[label].append(points[i])
            else:
                newLabel = 'outlier'
                if newLabel not in clusters:
                    clusters[newLabel] = []
                clusters[newLabel].append(points[i])

        centroids = []

        for label, points in clusters.items():

            if label == 'outlier':
                for point in points:
                    centroids.append(point)
            else:

                points_arr = np.array(points)

                centroid = np.mean(points_arr[:,:2],axis=0)
                centroid = np.append(centroid,points_arr[0,2])
                #self.get_logger().info(str(centroid))
                #self.get_logger().info("helooa")
                centroids.append(tuple(centroid))

        #Save latest centroids for next clustering

        self.map = centroids
        #self.get_logger().info(str(centroids))

        """
        for centroid in centroids:
            self.get_logger().info(str(centroid))
            self.map[math.floor(centroid[0]*100/20)][math.floor(centroid[1]*100/20)] = centroid[2]
        """

    def publish_map(self):
        points = []

        # add position of turtlebot as point
        turtlestate = TurtlebotState()
        turtlestate.x = self.turtle_state["x"]
        turtlestate.y = self.turtle_state["y"]
        turtlestate.angle = self.turtle_state["angle"]

        # add points of map
        for point in self.map:
            cp = ClassedPoint()
            cp.x = float(point[0])
            cp.y = float(point[1])
            cp.c = int(point[2])
            points.append(cp)

        point_array = PointArray()
        point_array.data = points
        point_array.turtlestate = turtlestate

        polygon = Polygon()
        polygon.points = []
        for point in self.middlepoints:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            polygon.points.append(p)
        point_array.middlepoints = polygon

        self.publisher_pointarray.publish(point_array)
        #self.get_logger().info("Published points list")


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
