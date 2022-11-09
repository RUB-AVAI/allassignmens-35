import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_system_default
import curses


class Textfield():
    def __init__(self,stdscr,lines=12):
        self._screen = stdscr
        #self._screen.nodelay(True)

    def read(self):
        k = self._screen.getkey()
        return k


class Controller(Node):
    def __init__(self, window):
        super().__init__("controller")
        self._pub = self.create_publisher(Twist, "cmd_vel", qos_profile=qos_profile_system_default)
        #self._time = self.create_timer(0.2, key_callback())
        self._window = window
        self.active = True
        self.mov_keys = {
            "w": (1.0, 0.0),
            "s": (-0.5, 0.0),
            "a": (0.0, 1.0),
            "d": (0.0, -1.0)
        }

    def listen(self):
        while self.active:
            key = self._window.read()
            if key in self.mov_keys.keys():
                t = Twist()
                t.linear.x, t.angular.z = self.mov_keys[key]
                self._pub.publish(t)
                self.get_logger().info("send")
            if key == "q":
                self.active = False


def start(stdscr):
    rclpy.init()
    node = Controller(Textfield(stdscr))
    node.listen()

    node.destroy_node()
    rclpy.shutdown()


def main():
    curses.wrapper(start)


if __name__ == "__main__":
    main()