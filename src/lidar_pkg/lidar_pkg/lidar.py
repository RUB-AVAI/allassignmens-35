import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from avai_messages.msg import FloatArray
from avai_messages.msg import FloatList
from rclpy.qos import qos_profile_sensor_data


def lin_map(x):
    #return 1/(212 - 148) * (x-148)
    return 1/63*x


def filter_scan(scan: LaserScan):
    for r in range(scan.ranges):
        if scan.ranges[r] != 0 and scan.ranges[r-1] == 0 and scan.ranges[r+1] == 0:
            scan.ranges[r] = 0


def cluster(scan: LaserScan):
    fov = scan.ranges[149:212]
    index = -1
    error = 0.05
    last = 0
    clusters = []
    results = []
    for angle, distance in enumerate(fov):
        if abs(distance - last) > distance * error and distance != 0:
            clusters.append((angle, angle))
            last= distance
            index += 1
        elif distance != 0:
            start, end = clusters[index]
            clusters[index] = (start, angle)
            last = distance
    return clusters


class LidarFusion(Node):
    def __init__(self):
        super().__init__("lidar_fusion")

        self.neg_angle = -32
        self.pos_angle = 32
        self.lidar_scan = None
        self.detected_cones = []

        self.create_subscription(LaserScan, "scan", self.lidar_callback, qos_profile_sensor_data)
        self.create_subscription(FloatArray, "boundingboxes", self.bounding_callback, 10)

    def lidar_callback(self, msg):

        self.lidar_scan = msg
        filter_scan(self.lidar_scan)
        self.lidar_scan = cluster(self.lidar_scan)

        self.get_logger().info(f'angle {str(msg.ranges.index(range))} ranges   {str(range)}')

    def bounding_callback(self,msg):
        fused = []
        for obj in self.lidar_scan:
            angle, dist = obj
            possible_box = lin_map(angle)
            for box in msg.data:
                if possible_box == box[2]:
                    fused.append((angle, dist, box[4]))
            #for scan in self.lidar_scan.ranges:

        self.get_logger().info(fused)

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



