import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from avai_messages.msg import PointArray
from std_msgs.msg import Float64MultiArray
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
        self.search_counter=0
        self.twist = self.create_publisher(Twist, "/cmd_vel", 10)
        # might want to change to slam_odom
        self.subscriber_pose = self.create_subscription(TransformStamped, "/turtlebot_pose", self.odom_callback, 10)
        # wherever we get the point list
        # self.create_subscription(None,"None",self.path_callback,10)
        self.subscriber_map = self.create_subscription(PointArray, "updated_points", self.map_callback, 10)
        self.publisher = self.create_publisher(Float64MultiArray, "target_point", 10)

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
        #self.get_logger().info("dddd")
        if self.target is not None:
            x, y = self.target

            position_x = 5-msg.transform.translation.x
            position_y = 5-msg.transform.translation.y
            self.get_logger().info(f' position{(position_x,position_y)}')
            # calculate distance to target
            distance = numpy.sqrt((x - position_x) ** 2
                                  + (y - position_y) ** 2)
            self.get_logger().info(str(distance))
            if distance < 0.02:
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
                move.linear.x = 0.03  # might want to change constant
                move.angular.z = sin(steering_angle-theta)

                self.get_logger().info(f'distance {distance} s_angle{steering_angle}, theta{theta} rad {rad}')
                self.get_logger().info(f' x{move.linear.x} ,z{move.angular.z}')

                self.twist.publish(move)
                self.search_counter=0
        #else:
        #    self.twist.publish(Twist())

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
        #self.get_logger().info(f'vec{math.degrees(vect_angle)} turt{turtlestate["angle"]}')


        x2 = cos(vect_angle) * 2 + turtlestate["x"]
        y2 = sin(vect_angle) * 2 + turtlestate["y"]
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
        self.get_logger().info(str(fronts))
        if len(yellow) == 0:
            self.get_logger().info("no yellow cones")
            self.twist.publish(Twist())
            self.search(1) # turn slowly

        elif len(blue) == 0:
            self.get_logger().info("no blue cones")
            self.twist.publish(Twist())
            self.search(0)  # turn slowly in the other direction
            #return
        else:
            yellow.sort(key=lambda x: x[1])
            blue.sort(key=lambda x: x[1])


            min_yellow = yellow[0][0]
            min_blue = blue[0][0]


            middle_point = ((min_yellow[0] - min_blue[0]) * 0.5 + min_blue[0],
                            (min_yellow[1] - min_blue[1]) * 0.5 + min_blue[1])
            self.get_logger().info(str(middle_point))
            if self.target == None:
                middle = []
                middle.append(middle_point[0])
                middle.append(middle_point[1])
                publish = Float64MultiArray()
                publish.data = middle
                self.publisher.publish(publish)
            self.target = middle_point

    def search(self,rotation):
        if self.search_counter<30:
            rotate = Twist()
            if rotation ==0:
                rotate.angular.z=0.1
            elif rotation ==1:
                rotate.angular.z=-0.1

            self.search_counter+=1
            self.twist.publish(rotate)





def main(args=None):
    rclpy.init(args=args)
    node = MovementController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
