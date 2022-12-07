import unittest

import rclpy
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, KeyCode
import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../remote_control")
#from src.Remote_Ayman.remote_control.remote_control.controller import Controller
from controller import Controller


class ControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        print("setup")
        self.cont = Controller()
        self.test_node = rclpy.create_node("test")
        self.t = []
        self.test_node.create_subscription(Twist, "cmd_vel", lambda x: self.t.append(x),
                                           qos_profile=qos_profile_system_default)

    def tearDown(self) -> None:
        self.cont.destroy_node()
        self.test_node.destroy_node()

    def test_button_press(self):

        self.assertEqual(0.,0.)

        self.cont.on_press(KeyCode.from_char("w"))
        rclpy.spin_once(self.test_node)

        msg = self.t.pop()
        self.assertIsInstance(msg, Twist)
        self.assertGreater(msg.linear.x, 0.)

        self.cont.on_press(KeyCode.from_char("s"))
        rclpy.spin_once(self.test_node)
        msg = self.t.pop()
        self.assertIsInstance(msg, Twist)
        self.assertLess(msg.linear.x, 0.)

        self.cont.on_press(KeyCode.from_char("a"))
        rclpy.spin_once(self.test_node)
        msg = self.t.pop()
        self.assertIsInstance(msg, Twist)
        self.assertGreater(msg.angular.z, 0.)

        self.cont.on_press(KeyCode.from_char("d"))
        rclpy.spin_once(self.test_node)
        msg = self.t.pop()
        self.assertIsInstance(msg, Twist)
        self.assertLess(msg.angular.z, 0.)

