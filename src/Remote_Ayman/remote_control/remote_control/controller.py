import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener
from rclpy.qos import qos_profile_system_default
import curses

key = "none"

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
        self.key = "none"
        self._pub = self.create_publisher(Twist, "cmd_vel", qos_profile=qos_profile_system_default)
        #self._time = self.create_timer(0.2, key_callback())
        self._window = window
        self.active = True
        self.mov_keys = {
            'w': (1.0, 0.0),
            's': (-0.5, 0.0),
            'a': (0.0, 1.0),
            'd': (0.0, -1.0),
            "none": (0.0, 0.0)
        }
        with Listener(
                on_press=self.on_press,
                on_release=self.on_release, suppress=True) as listener:
            listener.join()


    def on_press(self, key):
        if key.char in self.mov_keys.keys():
            t = Twist()

            t.linear.x, t.angular.z = self.mov_keys[key.char]
            self._pub.publish(t)
            self.get_logger().info("send")
            self.get_logger().info("send")


    def on_release(self, key):
        t = Twist()
        t.linear.x, t.angular.z = self.mov_keys["none"]
        self._pub.publish(t)

    def listen(self):
        while self.active:
            key = self.key
            self.get_logger().info("send")
            if key in self.mov_keys.keys():
                t = Twist()
                self.get_logger().info(key)
                t.linear.x, t.angular.z = self.mov_keys[key]
                self._pub.publish(t)
                self.get_logger().info("send")
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