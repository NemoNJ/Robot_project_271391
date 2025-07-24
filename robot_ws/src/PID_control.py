#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller_node')

        # รับจาก keyboard
        self.create_subscription(String, '/keyboard_input', self.keyboard_callback, 10)

        # รับ encoder
        self.create_subscription(Float32, '/left_encoder', self.left_encoder_callback, 10)
        self.create_subscription(Float32, '/right_encoder', self.right_encoder_callback, 10)

        # ส่ง pwm
        self.left_pwm_pub = self.create_publisher(Float32, '/left_pwm', 10)
        self.right_pwm_pub = self.create_publisher(Float32, '/right_pwm', 10)

        # Target speed (-255 to 255)
        self.linear_level = 0
        self.angular_level = 0
        self.step = 51

        self.left_target = 0.0
        self.right_target = 0.0
        self.left_actual = 0.0
        self.right_actual = 0.0

        # PID parameters
        self.kp = 1.0
        self.ki = 0.2
        self.kd = 0.05

        self.left_integral = 0.0
        self.right_integral = 0.0
        self.left_last_error = 0.0
        self.right_last_error = 0.0

        self.timer = self.create_timer(0.05, self.pid_control)  # 20Hz

    def keyboard_callback(self, msg: String):
        key = msg.data.lower()

        if key == 'w':
            self.linear_level = min(5, self.linear_level + 1)
        elif key == 's':
            self.linear_level = max(-5, self.linear_level - 1)
        elif key == 'a':
            self.angular_level = max(-5, self.angular_level - 1)
        elif key == 'd':
            self.angular_level = min(5, self.angular_level + 1)
        elif key == 'x':
            self.linear_level = 0
            self.angular_level = 0
        else:
            return

        self.left_target = (self.linear_level - self.angular_level) * self.step
        self.right_target = (self.linear_level + self.angular_level) * self.step

        self.get_logger().info(
            f"[KEY:{key}] L_target: {self.left_target} | R_target: {self.right_target}"
        )

    def left_encoder_callback(self, msg):
        self.left_actual = msg.data

    def right_encoder_callback(self, msg):
        self.right_actual = msg.data

    def pid_control(self):
        # LEFT
        err_l = self.left_target - self.left_actual
        self.left_integral += err_l
        der_l = err_l - self.left_last_error
        out_l = self.kp * err_l + self.ki * self.left_integral + self.kd * der_l
        self.left_last_error = err_l

        # RIGHT
        err_r = self.right_target - self.right_actual
        self.right_integral += err_r
        der_r = err_r - self.right_last_error
        out_r = self.kp * err_r + self.ki * self.right_integral + self.kd * der_r
        self.right_last_error = err_r

        # Clamp
        out_l = max(-255, min(255, out_l))
        out_r = max(-255, min(255, out_r))

        self.left_pwm_pub.publish(Float32(data=out_l))
        self.right_pwm_pub.publish(Float32(data=out_r))

        self.get_logger().info(
            f"PID | L: {self.left_actual:.1f}/{self.left_target:.1f} → {out_l:.1f} | R: {self.right_actual:.1f}/{self.right_target:.1f} → {out_r:.1f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
