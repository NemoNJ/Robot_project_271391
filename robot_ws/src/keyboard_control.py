#!/usr/bin/env python3

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import time

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher_node')

        self.keyboard_pub = self.create_publisher(String, '/keyboard_input', 10)
        self.vel_sub = self.create_subscription(Float32, '/velocity', self.vel_callback, 10)
        self.vel_level_sub = self.create_subscription(Float32, '/angular_velocity', self.velturn_callback, 10)

        self.latest_vel = 0.0
        self.latest_ang_vel = 0.0

        self.show_log()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def vel_callback(self, msg: Float32):
        self.latest_vel = msg.data

    def velturn_callback(self, msg: Float32):
        self.latest_ang_vel = msg.data

    def show_log(self):
        log_message = (
            "\n\n====== Keyboard Control Mode ======\n\n"
            "   [w] : FORWARD\n\n"
            "   [s] : BACKWARD\n\n"
            "   [a] : TURN_LEFT\n\n"
            "   [d] : TURN_RIGHT\n\n"
            "   [p] : STOP\n\n"
            "-------------------------------------\n"
        )
        self.get_logger().info(log_message)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            key = self.get_key()
            if key == 'p':
                self.get_logger().info("STOP command")
                break

            if key in ['w', 'a', 's', 'd']:
                msg = String()
                msg.data = key
                self.keyboard_pub.publish(msg)

                time.sleep(0.1)
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.latest_vel != 0.0 and self.latest_ang_vel != 0.0:
                  self.latest_vel = 0.0
                  self.latest_ang_vel = 0.0
                self.get_logger().info(
                    f"command : '{key}'  speed : {self.latest_vel} turnspeed : {self.latest_ang_vel} "
                )
def main():
    rclpy.init()
    node = KeyboardPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
