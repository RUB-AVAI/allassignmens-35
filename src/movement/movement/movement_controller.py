import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from avai_messages.msg import PointArray
import numpy
from math import atan2, copysign, radians, cos, sin, sqrt
import tf_transformations
from pynput.keyboard import Key, Listener


class MovementController(Node):
    def __init__(self):

        super().__init__("controller")

        self.target = None
        self.path=None
        self.twist = self.create_publisher(Twist,"/cmd_vel",10)
        # might want to change to slam_odom
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        # wherever we get the point list
       # self.create_subscription(None,"None",self.path_callback,10)
        self.subscriber_map = self.create_subscription(PointArray, "updated_points", self.map_callback, 10)

        listener = Listener(
                on_press=self.on_press,
                )
        listener.start()

    def on_press(self, key):
        if key.char == "q":
            stop = Twist()
            self.twist.publish(Twist)
            rclpy.shutdown()


    def path_callback(self,msg):

        self.path = msg.data
        self.target = self.path.pop()

    def odom_callback(self,msg):
        if self.target is not None:
            # calcualte distance to target
            distance = numpy.sqrt((self.target.pose.pose.position.x-msg.pose.pose.position.x)**2
                                  +(self.target.pose.pose.position.y-msg.pose.pose.position.y)**2)
            if distance < 1:
                # reached target
                if len(self.path) > 0:
                    self.target = self.path.pop()
                else:
                    self.target=None
                    self.twist.publish(Twist())
                    # no new points
            else:

                theta=tf_transformations.euler_from_quaternion([msg.pose.pose.orientaion.x,msg.pose.pose.orientaion.y,
                                                                msg.pose.pose.orientaion.w,msg.pose.pose.orientaion.z])
                steering_angle = atan2(self.target.pose.pose.position.y-msg.pose.pose.position.y,
                                     self.target.pose.pose.position.x-msg.pose.pose.position.x)

                move = Twist()
                move.linear.x = 0.2*distance #might want to change constant
                move.angular.z = steering_angle-theta

                self.twist.publish(move)

    def map_callback(self, msg):
        if len(msg.data) <= 0:
            return
        points = []
        for point in msg.data:
            points.append([point.x, point.y, point.c])
        turtlestate = {"x": msg.turtlestate.x, "y": msg.turtlestate.y, "angle": msg.turtlestate.angle}

        self.calculate_next_target(turtlestate, points)

    def calculate_next_target(self, turtlestate, point_map):
        distance_threshold = 1
        sign = lambda x: copysign(1, x)

        # get front points
        yellow = []
        blue = []
        # use a 90 degree vector to decide if a point is in front of the bot or not
        #vector goes from bot position to a point (x2,y2) in the direction of the vector
        vect_angle = radians(turtlestate["angle"] + 90)
        x2 = cos(vect_angle) * 2
        y2 = sin(vect_angle) * 2
        for point in point_map:
            #check if point is "left" or "right" of vector
            front = sign((x2 - turtlestate["x"]) * (point[1] - turtlestate["y"]) -
                         (y2 - turtlestate["y"]) * (point[0] - turtlestate["x"]))
            if front > 0:
                # only points with a distance < distance_threshold are valid
                distance = sqrt((point[0] - turtlestate["x"]) ** 2 + (point[1] - turtlestate["y"]) ** 2)
                if distance <= distance_threshold:
                    if point[2] == 0:
                        blue.append([point, distance])
                    if point[2] == 2:
                        yellow.append([point, distance])
        if len(yellow) == 0:
            return  # turn slowly
        if len(blue) == 0:
            return  # turn slowly in the other direction

        yellow.sort(key=lambda x: x[1])
        blue.sort(key=lambda x: x[1])

        min_yellow = yellow[0][0]
        min_blue = blue[0][0]

        middle_point = ((min_yellow[0] - min_blue[0]) * 0.5 + min_blue[0],
                        (min_yellow[1] - min_blue[1]) * 0.5 + min_blue[1])

        self.target = middle_point

def main(args=None):
    rclpy.init(args=args)
    node = MovementController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()