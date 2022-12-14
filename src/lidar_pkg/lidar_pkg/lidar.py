import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from avai_messages.msg import FloatArray
from avai_messages.msg import FloatList
from rclpy.qos import qos_profile_sensor_data



def lin_map(x):
    return 212 + (148 - 212) * x


class LidarFusion(Node):
    def __init__(self):
        super().__init__("lidar_fusion")

        self.neg_angle = -32
        self.pos_angle = 32
        self.lidar_scan = None
        self.detected_cones = []

        self.create_subscription(LaserScan, "scan", self.lidar_callback, qos_profile_sensor_data)
        self.create_subscription(FloatArray, "boundingboxes", self.bounding_callback, 10)

    def lidar_callback(self,msg):
        range = 1000.0
        self.lidar_scan = msg
        for i in msg.ranges:
            if i < range and i > 0.0:
                range = i

        self.get_logger().info(f'angle {str(msg.ranges.index(range))} ranges   {str(range)}')

    def bounding_callback(self,msg):

        for box in msg.list:
            lowest = 1000
            ranges = {}
            possible_angle = lin_map(box[1])
            #for scan in self.lidar_scan.ranges:
            self.lidar_scan.ranges[possible_angle]


              #  if possible_angle-1.0 <= scan.ang <= possible_angle+1.0:
              #      ranges.update({scan.ang:scan.range})
              #      if scan.range < lowest:
               #         lowest=scan.range
            #angle = {i for i in ranges if ranges[i] == lowest}
            #self.get_logger().info(f'found boxes: {box[7]} at angle{angle}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()



