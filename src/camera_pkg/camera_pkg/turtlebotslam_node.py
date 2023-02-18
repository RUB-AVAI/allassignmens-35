import rclpy
import tf2_ros

from rclpy.node import Node
from tf2_ros import TransformListener
from  tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class TurtleBotPosePublisher(Node):

    def __init__(self):
        super().__init__("slam_node")
        self.tf_buffer=Buffer()
       # self.node = rclpy.create_node('turtlebot_pose_publisher')
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.publisher = self.create_publisher(TransformStamped, '/turtlebot_pose', 10)
        self.subscriber_pose = self.create_subscription(Odometry, "/odom", self.save_header,10) # old
        self.run_timer_ = self.create_timer(0.5, self.run)
        self.current_header = None

    def save_header(self, msg):
        self.current_header = msg.header

    def run(self):
        #while rclpy.ok():
            try:
                trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=1.0))
                pose = TransformStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.current_header.stamp#self.get_clock().now().to_msg()
                pose.child_frame_id = 'base_link'
                pose.transform.translation.x = trans.transform.translation.x

                pose.transform.translation.y = trans.transform.translation.y
                pose.transform.translation.z = trans.transform.translation.z
                pose.transform.rotation.x = trans.transform.rotation.x
                pose.transform.rotation.y = trans.transform.rotation.y
                pose.transform.rotation.z = trans.transform.rotation.z
                pose.transform.rotation.w = trans.transform.rotation.w
                self.publisher.publish(pose)
                #rclpy.spin_once(self.node)
            except (tf2_ros.LookupException,tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
                #tf2_ros.LookupException,
                self.get_logger().info("error:")
                self.get_logger().info(str(err))
                #continue

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()