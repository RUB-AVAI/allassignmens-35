import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener
from rclpy.qos import qos_profile_system_default

key = "none"


class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.key = "none"
        self._pub = self.create_publisher(Twist, "cmd_vel", qos_profile=qos_profile_system_default)

        self.mov_keys = {
            'w': .10,
            's': -0.2,
            'a':  1.0,
            'd': -1.0,
            "none": 0.0
        }
        listener = Listener(
                on_press=self.on_press,
                on_release=self.on_release, suppress=False)
        listener.start()

    def on_press(self, key):
        try:
            if key.char in self.mov_keys.keys():
                t = Twist()
                if key.char in ["w", "s"]:

                    t.linear.x = self.mov_keys[key.char]

                elif key.char in ["a", "d"]:
                    t.angular.z = self.mov_keys[key.char]

                self._pub.publish(t)
                self.get_logger().info(str(key))
                self.get_logger().info("send")
            elif key.char == "q":
                rclpy.shutdown()
        except AttributeError:
            pass

    def on_release(self, key):
        t = Twist()
        t.linear.x = self.mov_keys["none"]
        t.angular.z = self.mov_keys["none"]
        self._pub.publish(t)



def main():

    rclpy.init()

    node = Controller()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()