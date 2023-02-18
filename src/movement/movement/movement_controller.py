import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy
from math import atan2
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


def main(args=None):
    rclpy.init(args=args)
    node = MovementController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()