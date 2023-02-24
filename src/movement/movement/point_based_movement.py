import rclpy
from rclpy.node import Node
from avai_messages.msg import FloatArray
from std_msgs.msg import Float64MultiArray
from math import sin,cos,sqrt, asin,radians,degrees

from geometry_msgs.msg import Twist

def distance(angle_dist1,angle_dist2):
    alpha,a=angle_dist1
    beta,b=angle_dist2
    return sqrt(a*a + b*b - 2*a*b*cos(radians(abs(31.5-alpha) + abs(31.5-beta))))

def seitenhalbierende(a,b,c):

    return sqrt(2 * (a**2 + b**2) - c**2) / 2

 #0.184

def pythago1(a,c):
    return sqrt((c**2)-(a**2))

class Movement(Node):
    def __init__(self):
        super().__init__("movement")
        self.create_subscription(FloatArray,"lidar_values",self.control,10)
        self.publisher=self.create_publisher(Float64MultiArray,"target_point",10)
        self.blue_cones=[]
        self.yellow_cones = []
        self.target_point=None
        self.count=0


    def points_callback(self,msg):
        self.count+=1
        for point in msg.lists:
            angle, distance, color = point.elements
            if color == 0:
                self.blue_cones.append((angle, distance))
            elif color == 2:
                self.yellow_cones.append((angle, distance))

        self.blue_cones.sort(key=lambda x:x[1])
        self.yellow_cones.sort(key=lambda x:x[1])

    def calculate_point(self,angle_dist1,angle_dist2):
        aplha,a=angle_dist1
        beta,b=angle_dist2
        d=distance(angle_dist1,angle_dist2)
        self.get_logger().info(f'd {str(d)}')
        s=seitenhalbierende(a,b,d)
        self.get_logger().info(f's {str(s)}')
        c=self.center(angle_dist1,angle_dist2)
        self.get_logger().info(f'c {str(c)}')

        dist_to_point=pythago1(c,s)   #c=0.184
        angle_to_point=degrees(asin(c/s))
        return (angle_to_point,dist_to_point,4.0)

    def center(self,angle_dist1, angle_dist2):
        alpha, a = angle_dist1
        beta, b = angle_dist2

        line_a = sin(radians(abs(30 - alpha))) * a  # 0,184
        line_b = sin(radians(abs(30 - beta)))*b   # 0.184

        self.get_logger().info(f'l_a {line_a},l_b{line_b}')

        return abs(line_a -line_b)
    def control(self,msg):
           # while self.count<10:
            #    self.points_callback(msg)

            self.count=0
            first_blue=self.blue_cones[0]
            first_yellow=self.yellow_cones[0]
            self.get_logger().info(f'blue:{str(first_blue)} yellow:{str(first_yellow)}')
            angle,dist,color =self.calculate_point(first_blue,first_yellow)
            if(first_blue[1]>first_yellow[1]):
                if(first_blue[0]>first_yellow[0]):
                    angle=30+angle
                else:
                    angle:30-angle
            else:
                if (first_blue[0] < first_yellow[0]):
                    angle = 30 + angle
                else:
                    angle: 30 - angle
            target_point=(angle,dist,color)
            publish =Float64MultiArray()
            publish.data=target_point

            self.get_logger().info(str(target_point))
            self.publisher.publish(publish)


def main(args=None):
    rclpy.init(args=args)
    node = Movement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

















