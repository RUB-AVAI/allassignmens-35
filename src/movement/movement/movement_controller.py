import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from avai_messages.msg import PointArray
import numpy
from math import atan2, copysign, radians, cos, sin, sqrt
import tf_transformations
from pynput.keyboard import Key, Listener
from geometry_msgs.msg import TransformStamped



class MovementController(Node):
    def __init__(self):

        super().__init__("controller")

        self.target = None
        self.path = None
        self.twist = self.create_publisher(Twist, "/cmd_vel", 10)
        # might want to change to slam_odom
        self.subscriber_pose = self.create_subscription(TransformStamped, "/turtlebot_pose", self.odom_callback, 10)
        # wherever we get the point list
        # self.create_subscription(None,"None",self.path_callback,10)
        self.subscriber_map = self.create_subscription(PointArray, "updated_points", self.map_callback, 10)

        listener = Listener(
            on_press=self.on_press,
        )
        listener.start()

    def on_press(self, key):
        try:
            if key.char == "p":
                stop = Twist()
                self.twist.publish(stop)
                rclpy.shutdown()
        except AttributeError:
            pass

    def path_callback(self, msg):

        self.path = msg.data
        self.target = self.path.pop()

    def odom_callback(self, msg):
        self.get_logger().info("dddd")
        if self.target is not None:
            x, y = self.target

            position_x = 5-msg.transform.translation.x
            position_y = 5-msg.transform.translation.y
            self.get_logger().info(f' position{(position_x,position_y)}')
            # calculate distance to target
            distance = numpy.sqrt((x - position_x) ** 2
                                  + (y - position_y) ** 2)
            self.get_logger().info(str(distance))
            if distance < 0.05:
                # reached target
                    self.target=None
                    #search

                    # no new points
            else:

                _ , _ , theta = tf_transformations.euler_from_quaternion([ msg.transform.rotation.x, msg.transform.rotation.y,
                                                                          msg.transform.rotation.z,msg.transform.rotation.w])
                theta+=math.pi
                steering_angle = atan2(y - position_y,x - position_x)
                rad = steering_angle-theta
                if rad > math.pi:
                    rad -= 2. * math.pi
                elif rad < -math.pi:
                    rad += 2. * math.pi

                move = Twist()
                move.linear.x = 0.05  # might want to change constant
                move.angular.z = sin(steering_angle-theta)

                self.get_logger().info(f'distance {distance} s_angle{steering_angle}, theta{theta} rad {rad}')
                self.get_logger().info(f' x{move.linear.x} ,z{move.angular.z}')

                self.twist.publish(move)
        else:
            self.twist.publish(Twist())
    def map_callback(self, msg):
        if len(msg.data) <= 0:
            return
        points = []
        for point in msg.data:
            points.append([point.x, point.y, point.c])
        turtlestate = {"x": msg.turtlestate.x, "y": msg.turtlestate.y, "angle": msg.turtlestate.angle}

        self.calculate_next_target(turtlestate, points)

    def calculate_next_target(self, turtlestate, point_map):
        distance_threshold = 10
        sign = lambda x: copysign(1, x)
        # self.get_logger().info(str(point_map))
        # get front points
        yellow = []
        blue = []
        # use a 90 degree vector to decide if a point is in front of the bot or not
        # vector goes from bot position to a point (x2,y2) in the direction of the vector
        vect_angle = radians(turtlestate["angle"]+90)
        self.get_logger().info(f'vec{math.degrees(vect_angle)} turt{turtlestate["angle"]}')


        x2 = cos(vect_angle) * 2
        y2 = sin(vect_angle) * 2
        fronts = []
        for point in point_map:
            # check if point is "left" or "right" of vector
            front = sign((x2 - turtlestate["x"]) * (point[1] - turtlestate["y"]) -
                         (y2 - turtlestate["y"]) * (point[0] - turtlestate["x"]))
            if front < 0:
                fronts.append(point)
                # only points with a distance < distance_threshold are valid
                distance = sqrt((point[0] - turtlestate["x"]) ** 2 + (point[1] - turtlestate["y"]) ** 2)
                if distance <= distance_threshold:
                    if point[2] == 0:
                        blue.append([point, distance])
                    if point[2] == 2:
                        yellow.append([point, distance])
        # if len(yellow) == 0:
        #   return  # turn slowly
        # if len(blue) == 0:
        #   return  # turn slowly in the other direction

        self.get_logger().info(str(fronts))
        yellow.sort(key=lambda x: x[1])
        blue.sort(key=lambda x: x[1])


        min_yellow = yellow[0][0]
        min_blue = blue[0][0]


        middle_point = ((min_yellow[0] - min_blue[0]) * 0.5 + min_blue[0],
                        (min_yellow[1] - min_blue[1]) * 0.5 + min_blue[1])
        self.get_logger().info(str(middle_point))
        if self.target is None:
            self.target = middle_point


def main(args=None):
    rclpy.init(args=args)
    node = MovementController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
