#!/usr/bin/env python3

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import time
import select  # ใช้เช็คคีย์แบบไม่บล็อก

# BANNER = r"""
# --------------------------------------------------------------------------------------------

# RRRRR    EEEEE   AAAAA   III     CCCCC      OOOOO    M   M   EEEEE   BBBBB    AAAAA   CCCCC   K   K
# R    R   E       A   A    I     C          O     O   MM MM   E       B    B  A     A C       K  K 
# RRRRR    EEEE    AAAAA    I     C          O     O   M M M   EEEE    BBBBB   AAAAAAA C       KKK  
# R   R    E       A   A    I     C          O     O   M   M   E       B    B  A     A C       K  K 
# R    R   EEEEE   A   A   III     CCCCC      OOOOO    M   M   EEEEE   BBBBB   A     A  CCCCC  K   K

# --------------------------------------------------------------------------------------------
# """

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

        # Parameters
        self.plant_count = 1
        self.linear_level = 0
        self.angular_level = 0
        self.auto_plant = 0
        self.step = 102         # Velocity step per level
        self.step_turn = 70
        self.max_speed = 240    # Maximum abs speed
        self.prev_key = ''

        # ค่าแสดงผลล่าสุด (ที่ publish ไปแล้ว) สำหรับโชว์บนหน้าจอ
        self.last_linear = 0.0
        self.last_angular = 0.0

        # เก็บบรรทัดสถานะล่าสุด (log บนหน้าจอ) สูงสุด 6 บรรทัด
        self.status_lines = []

        # แสดงครั้งแรก
        self.render_screen()

    # ==== Utility: อ่านคีย์แบบไม่บล็อก ด้วย timeout วินาที ====
    def get_key(self, timeout=0.05):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)  # โหมดคีย์ทันที
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                return sys.stdin.read(1)
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # ==== Utility: เพิ่มบรรทัดสถานะ + log ลง rclpy ====
    def add_status(self, msg: str):
        # log ลง ROS ด้วย
        self.get_logger().info(msg)
        # เก็บเฉพาะสถานะล่าสุดเพียง 1 บรรทัด
        self.status_lines = [msg]

    # ==== หน้าจอหลัก: ล้างจอ + วาดใหม่ ====
    def render_screen(self):
        # ล้างหน้าจอและเลื่อนไปมุมซ้ายบน
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

        # # วาดแบนเนอร์
        # sys.stdout.write(BANNER)
        sys.stdout.write("--------------------------------------------------------------------------------------------\n")
        # บรรทัดสถานะความเร็ว
        sys.stdout.write(
            # f"Current speed: LinearLevel={self.linear_level}, AngularLevel={self.angular_level}\n"
            f"Published: Linear={self.last_linear:.1f}, Angular={self.last_angular:.1f}\n"
        )

        # เส้นคั่น
        sys.stdout.write("--------------------------------------------------------------------------------------------\n")

        # แสดงสถานะล่าสุด (ข้อความจาก add_status / คำสั่งต่าง ๆ)
        if self.status_lines:
            sys.stdout.write("Status:\n")
            for line in self.status_lines:
                sys.stdout.write(f" - {line}\n")
        else:
            sys.stdout.write("Status:\n - Ready.\n")

        sys.stdout.write("\n(Press Y to Stop & Exit)\n")
        sys.stdout.flush()

    # ==== คำนวณ/ส่งความเร็ว และอัปเดตสถานะ ====
    def calculate_and_publish_velocity(self, key):
        # ปรับเลเวลตามปุ่ม
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
            self.linear_level = 1
            self.angular_level = 0
        elif key == 'f':
            self.linear_level = -1
            self.angular_level = 0
        elif key == 'q':  # หมุนซ้าย
            self.angular_level = -1
            self.linear_level = 0
        elif key == 'e':  # หมุนขวา
            self.angular_level = 1
            self.linear_level = 0
        else:
            return False

        if key in ['s'] and self.prev_key in ['a', 'd', 'w', 'e', 'q']:
            self.linear_level = 0

        self.prev_key = key

        # สร้าง message
        linear_speed = Float32()
        angular_speed = Float32()

        # Linear
        if key in ['r', 'f']:
            raw_linear = self.linear_level * self.step / 2
        else:
            raw_linear = self.linear_level * self.step

        if raw_linear > 0:
            linear_speed.data = float(min(raw_linear, self.max_speed))
        else:
            linear_speed.data = float(max(raw_linear, -self.max_speed))

        # Angular
        if key in ['q', 'e']:
            raw_angular = self.angular_level * self.step_turn * 1.8
        else:
            raw_angular = self.angular_level * self.step_turn * 2

        if raw_angular > 0:
            angular_speed.data = float(min(raw_angular, self.max_speed))
        else:
            angular_speed.data = float(max(raw_angular, -self.max_speed))

        # Publish
        self.linear_pub.publish(linear_speed)
        self.angular_pub.publish(angular_speed)

        # เก็บค่าไว้โชว์หน้าจอ
        self.last_linear = linear_speed.data
        self.last_angular = angular_speed.data

        # เพิ่มสถานะบนหน้าจอ
        self.add_status(f"[KEY:{key}] Linear: {linear_speed.data:.1f} | Angular: {angular_speed.data:.1f}")

        return True

    # ==== คำสั่งช่วยหยุด ====
    def _zero_and_log(self, msg: str):
        zero = Float32()
        zero.data = 0.0
        self.linear_pub.publish(zero)
        self.angular_pub.publish(zero)
        self.add_status(msg)

    # ==== ลูปหลัก ====
    def run(self):
        # ลูปหลัก: หมุน ROS + อ่านคีย์ + รีเฟรชจอ
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            key = self.get_key(timeout=0.05)
            if key is not None:
                key = key.lower()

                # ออกจากโปรแกรม
                if key == 'y':
                    self._zero_and_log("STOP command received. Exiting...")
                    self.render_screen()
                    break

                # ชุดคำสั่ง Auto plant / โหลด / origin / reload / driller
                elif key == 'p':
                    self._zero_and_log("Planting garden")
                    self.auto_plant = 1
                    self.palnt_pub.publish(Int32(data=self.auto_plant))
                    self.render_screen()
                    time.sleep(3)
                    self.auto_plant = 0
                    self.palnt_pub.publish(Int32(data=self.auto_plant))
                    self.add_status("Planting completed.")
                elif key == 'o':
                    self._zero_and_log("Planting load")
                    self.auto_plant = 2
                    self.palnt_pub.publish(Int32(data=self.auto_plant))
                    self.render_screen()
                    time.sleep(3)
                    self.auto_plant = 0
                    self.palnt_pub.publish(Int32(data=self.auto_plant))
                    self.add_status("Load Completed")
                elif key == 'i':
                    self._zero_and_log("Planting garden...origin")
                    self.auto_plant = 3
                    self.palnt_pub.publish(Int32(data=self.auto_plant))
                    self.render_screen()
                    time.sleep(3)
                    self.auto_plant = 0
                    self.palnt_pub.publish(Int32(data=self.auto_plant))
                    self.add_status("Origin completed.")
                elif key == 'l':
                    self._zero_and_log("Planting garden...reload")
                    self.auto_plant = 4
                    self.palnt_pub.publish(Int32(data=self.auto_plant))
                    self.render_screen()
                    time.sleep(3)
                    self.auto_plant = 0
                    self.palnt_pub.publish(Int32(data=self.auto_plant))
                    self.add_status("Reload completed.")
                elif key == 'u':
                    self._zero_and_log("Driller ...")
                    self.auto_plant = 5
                    self.palnt_pub.publish(Int32(data=self.auto_plant))
                    self.render_screen()
                    time.sleep(3)
                    self.auto_plant = 0
                    self.palnt_pub.publish(Int32(data=self.auto_plant))
                    self.add_status("Driller completed.")

                # กลุ่มแมนนวล/กล้อง/คาร์ทีเซียน
                elif key == 'n':
                    self.gripper_man_pub.publish(Int32(data=1))
                    self.add_status("Gripper move: Occupied")
                elif key == 'm':
                    self.gripper_man_pub.publish(Int32(data=0))
                    self.add_status("Gripper move: Non_Occupied")
                elif key == 't':
                    self.cartesian_man_pub.publish(Int32(data=1))
                    self.add_status("Cartesian move x153 203 : test")
                elif key == 'v':
                    self.cartesian_man_pub.publish(Int32(data=0))
                    self.add_status("Anti_Vibration")
                elif key == 'k':
                    self.camera_pub.publish(Int32(data=1))
                    self.add_status("Camera move: +")
                elif key == 'j':
                    self.camera_pub.publish(Int32(data=0))
                    self.add_status("Camera move: -")

                # กลุ่มควบคุมความเร็ว
                if key in ['w', 'a', 's', 'd', 'c', 'r', 'q', 'e', 'f']:
                    if self.calculate_and_publish_velocity(key):
                        # หน่วงเล็กน้อยกันคีย์ค้าง
                        time.sleep(0.05)

            # รีเฟรชหน้าจอเสมอ (อัปเดตตลอดเวลา)
            self.render_screen()

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
