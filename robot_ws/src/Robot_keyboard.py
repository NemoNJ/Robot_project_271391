#!/usr/bin/env python3

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import time

class KeyboardVelocityPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_velocity_publisher_node')
        # Publishers for velocity commands
        self.linear_pub = self.create_publisher(Float32, '/velocity', 10)
        self.angular_pub = self.create_publisher(Float32, '/angular_velocity', 10)
        self.palnt_pub = self.create_publisher(Int32, '/auto_plant', 10)
        self.camera_pub = self.create_publisher(Int32, '/move_camera', 10)
        self.gripper_man_pub = self.create_publisher(Int32, '/move_gripper', 10)
        self.cartesian_man_pub = self.create_publisher(Int32, '/move_cartesian', 10)
        # Velocity parameters]
        self.plant_count = 1
        self.linear_level = 0
        self.angular_level = 0
        self.auto_plant = 0
        self.step = 102  # Velocity step per level
        self.step_turn = 70
        self.max_speed = 240  # Maximum speed (both positive and negative)
        self.prev_key = ''
        
        self.show_instructions()
    
    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    def show_instructions(self):
        instructions = (
            "\n\n====== Keyboard Control Mode ======\n\n"
            "   [w] : Increase forward speed\n\n"
            "   [s] : Increase backward speed\n\n"
            "   [a] : Increase left turn (skid steer) speed\n\n"
            "   [d] : Increase right turn (skid steer) speed\n\n"
            "   [c] : Brake\n\n"
            "   [r] : slow move\n\n"
            "   [p] : auto_plant\n\n"
            "   [k] : emergency_plant_mannual\n\n"
            "   [j] : emergency_gripper_mannual\n\n"
            "   [l] : emergency_load_mannual\n\n"
            "   [o] : STOP and exit\n\n"
            "-------------------------------------\n"
            f"Current speed: Linear={self.linear_level}, Angular={self.angular_level}\n"
        )
        self.get_logger().info(instructions)
    
    def calculate_and_publish_velocity(self, key):
        # Handle key presses
        if key == 'w':
            self.linear_level = min(5, self.linear_level + 1)
            self.angular_level = 0
        elif key == 's':
            self.linear_level = max(-5, self.linear_level - 1)
            self.angular_level = 0
        elif key == 'a':
            self.angular_level = max(-5, self.angular_level - 1)
            self.linear_level = 0
        elif key == 'd':
            self.angular_level = min(5, self.angular_level + 1)
            self.linear_level = 0
        elif key == 'c':
            self.angular_level = 0
            self.linear_level = 0
        elif key == 'r':
            self.linear_level = 1 if self.linear_level >= 0 else -1
            self.angular_level = 0
        elif key == 'q':  # หมุนซ้าย
            self.angular_level = -1
            self.linear_level = 0
        elif key == 'e':  # หมุนขวา
            self.angular_level = 1
            self.linear_level = 0
        else:
            return False
        
        # Reset angular level if switching from linear movement
        # if key in ['a'] and self.prev_key in ['w', 's' , 'd']:
        #     self.angular_level = 0
        
        # if key in ['d'] and self.prev_key in ['w', 's' , 'a']:
        #     self.angular_level = 0
        
        # # Reset linear level if switching from angular movement
        # if key in ['w'] and self.prev_key in ['a', 'd' , 's']:
        #     self.linear_level = 0
        
        if key in ['s'] and self.prev_key in ['a', 'd' , 'w' ,'e','q']:
            self.linear_level = 0
            
        # Update previous key
        self.prev_key = key
        
        # Create and publish velocity messages with speed limiting
        linear_speed = Float32()
        angular_speed = Float32()
        
        # Calculate linear speed with limit
        if key == 'r':
            raw_linear = self.linear_level * self.step / 2
        else:
            raw_linear = self.linear_level * self.step
            
        # Apply speed limit for both positive and negative values
        if raw_linear > 0:
            linear_speed.data = float(min(raw_linear, self.max_speed))
        else:
            linear_speed.data = float(max(raw_linear, -self.max_speed))
             
        # Calculate angular speed with limit
        
        if key in ['q', 'e']:
            raw_angular = self.angular_level * self.step_turn*1.8
        else:
            raw_angular = self.angular_level * self.step_turn*2
        # Apply speed limit for both positive and negative values
        if raw_angular > 0:
            angular_speed.data = float(min(raw_angular, self.max_speed))
        else:
            angular_speed.data = float(max(raw_angular, -self.max_speed))
        
        self.linear_pub.publish(linear_speed)
        self.angular_pub.publish(angular_speed)
        
        self.get_logger().info(
            f"[KEY:{key}] Linear: {linear_speed.data} | Angular: {angular_speed.data}"
        )
        
        return True
    
    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            key = self.get_key().lower()
            
            if key == 'y':
                zero_vel = Float32()
                zero_vel.data = 0.0
                self.linear_pub.publish(zero_vel)
                self.angular_pub.publish(zero_vel)
                self.get_logger().info("STOP command received. Exiting...")
                break
            if key == 'p':
                zero_vel = Float32()
                zero_vel.data = 0.0
                self.linear_pub.publish(zero_vel)
                self.angular_pub.publish(zero_vel)
                self.auto_plant = 1
                self.palnt_pub.publish(Int32(data=self.auto_plant))
                self.get_logger().info(f"Planting garden")

                time.sleep(3)
                self.auto_plant = 0
                self.palnt_pub.publish(Int32(data=self.auto_plant))
                self.get_logger().info("Planting completed.")
            elif key == 'o':
                zero_vel = Float32()
                zero_vel.data = 0.0
                self.linear_pub.publish(zero_vel)
                self.angular_pub.publish(zero_vel)
                self.auto_plant = 2
                self.palnt_pub.publish(Int32(data=self.auto_plant))
                self.get_logger().info(f"Planting load")

                time.sleep(3)
                self.auto_plant = 0
                self.palnt_pub.publish(Int32(data=self.auto_plant))
                self.get_logger().info("Load Completed")
            elif key == 'i':
                zero_vel = Float32()
                zero_vel.data = 0.0
                self.linear_pub.publish(zero_vel)
                self.angular_pub.publish(zero_vel)
                self.auto_plant = 3
                self.palnt_pub.publish(Int32(data=self.auto_plant))
                self.get_logger().info(f"Planting garden...origin")

                time.sleep(3)
                self.auto_plant = 0
                self.palnt_pub.publish(Int32(data=self.auto_plant))
                self.get_logger().info("Origin completed.")
            elif key == 'l':
                zero_vel = Float32()
                zero_vel.data = 0.0
                self.linear_pub.publish(zero_vel)
                self.angular_pub.publish(zero_vel)
                self.auto_plant = 4
                self.palnt_pub.publish(Int32(data=self.auto_plant))
                self.get_logger().info(f"Planting garden...reload")

                time.sleep(3)
                self.auto_plant = 0
                self.palnt_pub.publish(Int32(data=self.auto_plant))
                self.get_logger().info("Reload completed.")
            elif key == 'u':
                zero_vel = Float32()
                zero_vel.data = 0.0
                self.linear_pub.publish(zero_vel)
                self.angular_pub.publish(zero_vel)
                self.auto_plant = 5
                self.palnt_pub.publish(Int32(data=self.auto_plant))
                self.get_logger().info(f"Driller ...")

                time.sleep(3)
                self.auto_plant = 0
                self.palnt_pub.publish(Int32(data=self.auto_plant))
                self.get_logger().info("Driller completed.")
            if key == 'm':
                self.gripper_man_pub.publish(Int32(data=1))
                self.get_logger().info("Gripper move: Occupied")
            if key == 'n':
                self.gripper_man_pub.publish(Int32(data=0))
                self.get_logger().info("Gripper move: Non_Occupied")
            if key == 't':
                self.cartesian_man_pub.publish(Int32(data=1))
                self.get_logger().info("Cartesian move x153 203 : test")
            if key == 'f':
                self.cartesian_man_pub.publish(Int32(data=0))
                self.get_logger().info("Anti_Vibration")
            if key == 'j':
                self.camera_pub.publish(Int32(data=1))
                self.get_logger().info("Camera move: 1")

            if key == 'k':
                self.camera_pub.publish(Int32(data=0))
                self.get_logger().info("Camera move: 0")
            if key in ['w', 'a', 's', 'd','c','r','q','e']:
                if self.calculate_and_publish_velocity(key):
                    time.sleep(0.1)  # Small delay to prevent rapid key repeats

def main():
    rclpy.init()
    node = KeyboardVelocityPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Publish zero velocities before shutting down
        zero_vel = Float32()
        zero_vel.data = 0.0
        node.linear_pub.publish(zero_vel)
        node.angular_pub.publish(zero_vel)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()