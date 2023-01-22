#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from avai_messages.msg import PointArray, ClassedPoint, FloatArray, FloatList, Point, Polygon, Polylines
from nav_msgs.msg import Odometry
import math
from typing import Tuple, List
from sklearn.cluster import DBSCAN
import numpy as np
import message_filters
import tf_transformations
from math import atan2

MAP_SIZE = 51  # map is 10.2m x 10.2m
CAMERA_RANGE = 60  # camera field of view in degrees

#NEED TO CHECK BOUNDS? Maybe replace array with list

class OccupancyMapNode(Node):
    def __init__(self):
        super().__init__("occupancy_map")

        # one cell is 20cm x 20cm
        # map[x][y]
        self.map = [[-1] * MAP_SIZE for x in range(MAP_SIZE)]
        # start position (meter) in self.map and angle in degrees
        self.turtle_state = {"x": float(MAP_SIZE*20/2/100), "y": float(MAP_SIZE*20/2/100), "angle": 0}
        self.turtle_state_is_set = False
        self.latest_centroids = []
        self.publisher_pointarray = self.create_publisher(PointArray, "updated_points", 10)
        self.publisher_polylines = self.create_publisher(Polylines, "polylines", 10)
        #self.subscriber_lidar = self.create_subscription(FloatArray,"lidar_values",self.callback_lidar_values,10)
        self.subscriber_lidar = message_filters.Subscriber(self, FloatArray, "lidar_values")
        self.subscriber_pose = message_filters.Subscriber(self, Odometry, "/odom")

        self.ts = message_filters.ApproximateTimeSynchronizer([self.subscriber_lidar, self.subscriber_pose], 100, 0.2)
        self.ts.registerCallback(self.callback_synchronised)

        self.get_logger().info("Occupancy Map started.")

    def callback_synchronised(self, msg_lidar, msg_odom):
        self.callback_pose(msg_odom)
        self.callback_lidar_values(msg_lidar)

    def callback_pose(self, msg):
        # convert quaternion to euler angle
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        r,p,y = \
            tf_transformations.euler_from_quaternion\
                ([qw, qx, qy, qz])
        #self.get_logger().info(f"position x{msg.pose.pose.position.x}, y{msg.pose.pose.position.y}, r{r}")
        #update turtle_state
        self.turtle_state["angle"] = math.degrees(-r)  # was (-3.14, 3.14), now counter clockwise 360
        #self.get_logger().info(f"{self.turtle_state['angle']}")
        self.turtle_state["x"] = \
            float(MAP_SIZE*20/100/2)+msg.pose.pose.position.x
        self.turtle_state["y"] = \
            float(MAP_SIZE*20/100/2)+msg.pose.pose.position.y
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
        :return: Dictionary in the format (x-coordinate, y-coordinate, classID). All coordinates are real world
        coordinates in meters.
        """
        x = math.cos(math.radians(data[0] + self.turtle_state["angle"] + CAMERA_RANGE/2 + 90))*data[1] \
            + self.turtle_state["x"]
        y = math.sin(math.radians(data[0] + self.turtle_state["angle"] + CAMERA_RANGE/2 + 90))*data[1] \
            + self.turtle_state["y"]
        self.get_logger().info(f"lidar: {data[0]} {data[1]} {data[2]}")
        return {"x": x, "y": y, "classID": data[2]}

    def update_map(self, positions: List[Tuple[float, float, int]]):
        """
        Places the classes of all lidar point in a given list into the map.
        :param positions: A list of tuples, representing the lidar points in the format (angle, distance, classID).
        """
        #for position in positions:
            #xyc = self.lidar_to_xy(position)
            #self.get_logger().info(f"xyc: {xyc}")

           # self.map[math.floor(xyc["x"]*100/20)][math.floor(xyc["y"]*100/20)] = xyc["classID"]

        points = self.latest_centroids
        for position in positions:
            xyc = self.lidar_to_xy(position)
            points.append([xyc["x"],xyc["y"],xyc["classID"]])

        xycoordinates = np.array(points)
        xycoordinates = xycoordinates[:,:2]
        dbscan = DBSCAN(eps=0.3, min_samples=2)
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
                self.get_logger().info(str(centroid))
                self.get_logger().info("helooa")
                centroids.append(centroid)

        #Reset Map

        self.map = [[-1] * MAP_SIZE for x in range(MAP_SIZE)]

        #Save latest centroids for next clustering

        self.latest_centroids = centroids
        self.get_logger().info(str(centroids))

        for centroid in centroids:
            self.get_logger().info(str(centroid))
            self.map[math.floor(centroid[0]*100/20)][math.floor(centroid[1]*100/20)] = centroid[2]

        self.publish_map()

    def publish_map(self):
        points = []

        # add position of turtlebot as point
        turtle_point = ClassedPoint()
        turtle_point.x = self.turtle_state["x"]
        turtle_point.y = self.turtle_state["y"]
        turtle_point.c = 3
        points.append(turtle_point)

        # add points of map
        for x in range(len(self.map)):
            for y in range(len(self.map[0])):
                if self.map[x][y] != -1:
                    p = ClassedPoint()
                    p.x = float(x*20/100)
                    p.y = float(y*20/100)
                    p.c = int(self.map[x][y])

                    points.append(p)

        point_array = PointArray()
        point_array.data = points

        self.publisher_pointarray.publish(point_array)
        self.get_logger().info("Published points list")

    def map_to_point_lists(self):
        blue = []
        orange = []
        yellow = []
        for x in range(len(self.map)):
            for y in range(len(self.map[0])):
                c = self.map[x][y]
                if c == -1:
                    continue
                if c == 0:
                    blue.append((x, y))
                if c == 1:
                    orange.append((x, y))
                if c == 2:
                    yellow.append((x, y))
        return blue, orange, yellow

    def publish_polylines(self):
        blue, orange, yellow = self.map_to_point_lists()
        # might need to convert coordinates to float in map_to_point_lists()
        polylines = Polylines()

        to_polyline(blue)
        polygon = Polygon()
        for point in blue:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            polygon.points.append(p)
        polylines.blue = polygon

        to_polyline(yellow)
        polygon = Polygon()
        for point in yellow:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            polygon.points.append(p)
        polylines.yellow = polygon
        self.publisher_polylines.publish(polylines)


def to_polyline(points: List):
    """
    Sorts a point list inplace, so that the points are in a circle.
    First point is equal to the last point.
    :param points: list of points to be modified
    """
    # get poiont with smallest x and y value
    points.sort(key=lambda x: [x[1], x[0]])
    p0 = points.pop(0)
    # sort points counter-clockwise by their angle relative to p0
    points.sort(key=lambda p: atan2(p[1]-p0[1], p[0]-p0[0]))  # atan2() is > 0, because all points are above p0
    points.insert(0, p0)
    points.append(p0)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
