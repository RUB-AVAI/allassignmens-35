import rclpy
import tf2_ros
from tf2_ros import TransformListener
from geometry_msgs.msg import TransformStamped

class TurtleBotPosePublisher:
    def init(self):
        self.node = rclpy.create_node('turtlebot_pose_publisher')
        self.tf_buffer = TransformListener(self.node)
        self.publisher = self.node.create_publisher(TransformStamped, '/turtlebot_pose', 10)

    def run(self):
        while rclpy.ok():
            try:
                trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.Time(0))
                pose = TransformStamped()
                pose.header.frame_id = 'map'
                pose.child_frame_id = 'base_link'
                pose.transform.translation.x = trans.transform.translation.x
                pose.transform.translation.y = trans.transform.translation.y
                pose.transform.translation.z = trans.transform.translation.z
                pose.transform.rotation.x = trans.transform.rotation.x
                pose.transform.rotation.y = trans.transform.rotation.y
                pose.transform.rotation.z = trans.transform.rotation.z
                pose.transform.rotation.w = trans.transform.rotation.w
                self.publisher.publish(pose)
                rclpy.spin_once(self.node)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

def main(args=None):
    rclpy.init(args=args)
    turtlebot_pose_publisher = TurtleBotPosePublisher()
    turtlebot_pose_publisher.run()

if __name__ == "__main__":
    main()