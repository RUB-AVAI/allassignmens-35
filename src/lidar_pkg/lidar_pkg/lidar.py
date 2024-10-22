import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from avai_messages.msg import FloatArray
from avai_messages.msg import FloatList
from rclpy.qos import qos_profile_sensor_data
import message_filters


def lin_map(x):
    # return 1/(212 - 148) * (x-148)
    if x < 30:
        return 1 - (1 / 62 * x)
    else:
        return 1 - (1 / 56 * x)


# 48,5 25,8 6,3
def filter_scan(scan: LaserScan):
    for r in range(len(scan.ranges)):
        # if r >= 1 and scan.ranges[r] != 0 and scan.ranges[r-1] == 0 and scan.ranges[r+1] == 0:
        #    scan.ranges[r] = 0
        if scan.ranges[r] > 1:
            scan.ranges[r] = 0


def cluster(scan: LaserScan):
    fov = scan.ranges[147:203]

    index = -1
    error = 0.1
    last = 0
    clusters = []

    for angle, distance in enumerate(fov):

        if abs(distance - last) > distance*0.1 and distance != 0:
            clusters.append((angle, angle, distance))
            last = distance
            index += 1
        elif distance != 0 and abs(distance - last) <= (distance * error):
            start, end, dist = clusters[index]
            clusters[index] = (start, angle, dist + distance)
            last = distance

    for x in range(len(clusters)):
        start, end, dist = clusters[x]
        clusters[x] = (start, end, (dist / ((end - start) + 1)))
    return clusters


class LidarFusion(Node):
    def __init__(self):
        super().__init__("lidar_fusion")

        self.neg_angle = -32
        self.pos_angle = 32
        self.lidar_scan = None
        self.detected_cones = []

        self.lidar_sub = message_filters.Subscriber(self, LaserScan, "scan", qos_profile=qos_profile_sensor_data)
        self.image_sub = message_filters.Subscriber(self, FloatArray, "bounding_box")
        ts = message_filters.ApproximateTimeSynchronizer([self.lidar_sub, self.image_sub], 50, 0.1, False)
        ts.registerCallback(self.bounding_callback)
        # self.create_subscription(LaserScan, "scan", self.lidar_callback, qos_profile_sensor_data)
        # self.create_subscription(FloatArray, "bounding_box", self.bounding_callback, 10)
        self.publisher = self.create_publisher(FloatArray, "lidar_values", 10)
        # self.create_timer(10,self.bounding_callback)

    """"
    def lidar_callback(self, msg):

        self.lidar_scan = msg

        #self.get_logger().info(str(self.lidar_scan.ranges[148:212]))
        filter_scan(self.lidar_scan)
        self.lidar_scan = cluster(self.lidar_scan)

       # self.get_logger().info(f'angle {str(msg.ranges.index(range))} ranges   {str(range)}')
    """

    def bounding_callback(self, lidar, msg):
        self.lidar_scan = lidar

        # filter_scan(self.lidar_scan)
        self.lidar_scan = cluster(self.lidar_scan)

        processed_bounding_box = []
        for lst in msg.lists:
            box = []
            for e in lst.elements:
                box.append(e)
                processed_bounding_box.append(box)
        fused_one = []
        fused = []

        for b in processed_bounding_box:
            if b[3] > 0.25:
                for obj in self.lidar_scan:
                    angle_s, angle_e, dist, = obj
                    possible_box = lin_map((angle_s + angle_e) / 2)
                    # self.get_logger().info(str(possible_box) + " " + str(dist))

                    if abs(possible_box - b[0]) < 0.04:
                        if ((angle_s + angle_e) / 2, dist, b[5]) not in fused_one:
                            if len(fused_one) == 0:
                                fused_one.append(((angle_s + angle_e) / 2, dist, b[5]))
                            elif dist < fused_one[0][1]:
                                fused_one[0] = ((angle_s + angle_e) / 2, dist, b[5])
                if len(fused_one) == 1 and not (fused_one[0] in fused):
                    fused.append(fused_one[0])
        """
        for obj in self.lidar_scan:
            angle_s,angle_e, dist, = obj
            possible_box = lin_map((angle_s+angle_e)/2)
            self.get_logger().info(str(possible_box) + " "+str(dist))
        
            
            
            for b in processed_bounding_box:
                if abs(possible_box - b[0]) < 0.04:
                    if((angle_s+angle_e)/2, dist, b[5]) not in fused_one:
                        if(len(fused_one)==0):
                            fused_one.append(((angle_s+angle_e)/2, dist, b[5]))
                        elif(dist<fused_one[0][1]):
                            fused_one[0]=((angle_s+angle_e)/2, dist, b[5])
                if(len(fused_one)==1):
                    fused.append(fused_one[0])
            #for scan in self.lidar_scan.ranges:
            """
        # only accept cones with distance < distance
        distance = 1.5
        fused = [p for p in fused if p[1] < distance]
        self.get_logger().info(str(self.lidar_scan))
        self.get_logger().info(str(fused))

        msg_lid = FloatArray()
        msg_lid.header.stamp = msg.header.stamp
        for ele in fused:
            lid_data = FloatList()
            lid_data.elements = ele
            msg_lid.lists.append(lid_data)

        self.publisher.publish(msg_lid)
        #  if possible_angle-1.0 <= scan.ang <= possible_angle+1.0:
        #      ranges.update({scan.ang:scan.range})
        #      if scan.range < lowest:
        #         lowest=scan.range
        # angle = {i for i in ranges if ranges[i] == lowest}
        # self.get_logger().info(f'found boxes: {box[7]} at angle{angle}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
