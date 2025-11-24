#!/usr/bin/env python3
import sys
import tty
import termios
import select
import shutil  # << ใช้หาขนาดหน้าจอ
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist   # << เพิ่มสำหรับ /robot_direction

# ถ้าต้องการ QoS เฉพาะ ให้เปิดคอมเมนต์บรรทัดด้านล่าง
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class KeyboardVelocityPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_velocity_publisher_node')

        # ---- Publishers ----
        # เดิม:
        # self.linear_pub = self.create_publisher(Float32, '/velocity', 10)
        # self.angular_pub = self.create_publisher(Float32, '/angular_velocity', 10)
        # ใหม่:
        self.direction_pub = self.create_publisher(Twist, '/robot_direction', 10)
        self.velocity_pub = self.create_publisher(Float32, '/robot_velocity', 10)

        self.palnt_pub = self.create_publisher(Int32, '/auto_plant', 10)
        self.camera_pub = self.create_publisher(Int32, '/move_camera', 10)
        self.gripper_man_pub = self.create_publisher(Int32, '/move_gripper', 10)
        self.cartesian_man_pub = self.create_publisher(Int32, '/move_cartesian', 10)

        # ---- Parameters / States ----
        self.plant_count = 1
        self.linear_level = 0
        self.angular_level = 0
        self.auto_plant = 0
        self.step = 102
        self.step_turn = 70
        self.max_speed = 240
        self.prev_key = ''

        # ค่า “คำสั่งล่าสุด” ที่จะถูกยิงซ้ำด้วย timer
        # ทิศทางเก็บเป็น Twist (-1,0,+1)
        self.current_direction = Twist()
        # ความเร็วเก็บเป็นขนาด (0..max_speed)
        self.current_velocity = 0.0

        # ค่าที่แสดงบนหน้าจอ
        # ใช้ last_linear/last_angular เพื่อเก็บค่าทิศทาง (±1, 0)
        self.last_linear = 0.0   # แทน direction.linear.x
        self.last_angular = 0.0  # แทน direction.angular.z
        self.last_velocity = 0.0
        self.status_lines = []

        # ---- ตั้ง RAW mode ให้คีย์เสถียร (ครั้งเดียว) ----
        self._fd = sys.stdin.fileno()
        self._old_tty = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)

        # ---- เข้า alternate screen + ซ่อน cursor + ปิด line wrap ----
        sys.stdout.write("\033[?1049h\033[?25l\033[?7l")
        sys.stdout.flush()

        # ---- Timers ----
        self.cmd_timer = self.create_timer(0.05, self._publish_hold)  # 20 Hz
        self.ui_timer = self.create_timer(0.1, self.render_screen)    # 10 Hz

        self._auto_timer = None
        self.render_screen()

    def destroy_node(self):
        try:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_tty)
        except Exception:
            pass
        sys.stdout.write("\033[?7h\033[?25h\033[?1049l")
        sys.stdout.flush()
        super().destroy_node()

    def get_key(self, timeout=0.0):
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None

    def add_status(self, msg: str):
        self.status_lines = [msg]

    # ==== วาดหน้าจอแบบกำหนดพิกัดให้ชิดซ้ายบน ====
    def render_screen(self):
        rows, cols = shutil.get_terminal_size(fallback=(24, 80))
        # ล้างจอและชี้เคอร์เซอร์ไปมุมซ้ายบน
        sys.stdout.write("\033[H\033[J")

        # แถวที่ 1 คอลัมน์ที่ 1: Published ...
        sys.stdout.write("\033[1;1H")  # row 1, col 1
        sys.stdout.write(
            f"Published: Dir.linear.x = {self.last_linear:.1f}, "
            f"Dir.angular.z = {self.last_angular:.1f}, "
            f"Speed = {self.last_velocity:.1f}\033[K"
        )

        # แถวที่ 3 คอลัมน์ที่ 1: หัวข้อ Status
        sys.stdout.write("\033[3;1HStatus:\033[K")

        # รายการสถานะ เริ่มแถวที่ 4 ชิดซ้ายทั้งหมด
        base_row = 4
        if self.status_lines:
            for i, line in enumerate(self.status_lines):
                sys.stdout.write(f"\033[{base_row+i};1H- {line}\033[K")
        else:
            sys.stdout.write(f"\033[{base_row};1H- Ready.\033[K")

        # บรรทัดคำแนะนำ วางล่างซ้าย (ชิดมุม)
        sys.stdout.write(f"\033[{rows};1H(Press Y to Stop & Exit)\033[K")

        sys.stdout.flush()

    def _publish_hold(self):
        # ยิงค่าคำสั่งล่าสุดซ้ำ
        self.direction_pub.publish(self.current_direction)
        self.velocity_pub.publish(Float32(data=self.current_velocity))

        # เก็บไว้โชว์บนหน้าจอ
        self.last_linear = self.current_direction.linear.x
        self.last_angular = self.current_direction.angular.z
        self.last_velocity = self.current_velocity

    def calculate_and_apply_velocity(self, key):
        # ปรับระดับตามปุ่มเหมือนเดิม
        if key == 'w':
            self.linear_level = min(5, self.linear_level + 1); self.angular_level = 0
        elif key == 's':
            self.linear_level = max(-5, self.linear_level - 1); self.angular_level = 0
        elif key == 'a':
            self.angular_level = max(-5, self.angular_level - 1); self.linear_level = 0
        elif key == 'd':
            self.angular_level = min(5, self.angular_level + 1); self.linear_level = 0
        elif key == 'c':
            self.angular_level = 0; self.linear_level = 0
        elif key == 'r':
            self.linear_level = 1; self.angular_level = 0
        elif key == 'f':
            self.linear_level = -1; self.angular_level = 0
        elif key == 'q':
            self.angular_level = -1; self.linear_level = 0
        elif key == 'e':
            self.angular_level = 1; self.linear_level = 0
        else:
            return False

        # logic เพิ่มเติมของเดิม
        if key in ['s'] and self.prev_key in ['a', 'd', 'w', 'e', 'q']:
            self.linear_level = 0
        self.prev_key = key

        # === คำนวณความเร็วดิบ (magnitude) ตามเดิม ===
        raw_linear = (self.linear_level * self.step / 2) if key in ['r', 'f'] else (self.linear_level * self.step)
        raw_angular = (self.angular_level * self.step_turn * 1.8) if key in ['q', 'e'] else (self.angular_level * self.step_turn * 2)

        # จำกัดความเร็วสูงสุด
        if raw_linear > 0:
            linear_mag = min(raw_linear, self.max_speed)
        else:
            linear_mag = max(raw_linear, -self.max_speed)

        if raw_angular > 0:
            angular_mag = min(raw_angular, self.max_speed)
        else:
            angular_mag = max(raw_angular, -self.max_speed)

        # === แยกเป็น direction (-1,0,+1) และ velocity (ขนาดบวก) ===
        direction_msg = Twist()
        speed_mag = 0.0

        # เงื่อนไขให้ linear กับ angular ไม่ทำงานพร้อมกัน
        if self.linear_level != 0:
            # เดินหน้า/ถอยหลัง
            direction_msg.linear.x = 1.0 if self.linear_level > 0 else -1.0
            direction_msg.angular.z = 0.0
            speed_mag = float(abs(linear_mag))
        elif self.angular_level != 0:
            # เลี้ยว
            # หมายเหตุ: ตามโจทย์ +1 เลี้ยวขวา, -1 เลี้ยวซ้าย
            direction_msg.linear.x = 0.0
            direction_msg.angular.z = 1.0 if self.angular_level > 0 else -1.0
            speed_mag = float(abs(angular_mag))
        else:
            # หยุด
            direction_msg.linear.x = 0.0
            direction_msg.angular.z = 0.0
            speed_mag = 0.0

        # เก็บค่าไว้เป็นคำสั่งล่าสุด
        self.current_direction = direction_msg
        self.current_velocity = speed_mag

        self.add_status(
            f"[KEY:{key}] Dir.linear.x: {direction_msg.linear.x:.1f} | "
            f"Dir.angular.z: {direction_msg.angular.z:.1f} | "
            f"Speed: {speed_mag:.1f}"
        )
        return True

    def _zero_and_log(self, msg: str):
        # หยุดหุ่นยนต์: ทิศทาง 0 ทั้งคู่ + ความเร็ว 0
        self.current_direction = Twist()
        self.current_velocity = 0.0
        self.add_status(msg)

    def _do_auto(self, mode: int, start_msg: str, done_msg: str, duration=3.0):
        self._zero_and_log(start_msg)
        self.auto_plant = mode
        self.palnt_pub.publish(Int32(data=self.auto_plant))

        def _finish():
            self.auto_plant = 0
            self.palnt_pub.publish(Int32(data=self.auto_plant))
            self.add_status(done_msg)
            if self._auto_timer is not None:
                self._auto_timer.cancel()
                self._auto_timer = None

        self._auto_timer = self.create_timer(duration, _finish)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            key = self.get_key(0.0)
            if not key:
                continue
            key = key.lower()

            if key == 'y':
                self._zero_and_log("STOP command received. Exiting...")
                break

            if key == 'p':
                self._do_auto(1, "Planting garden", "Planting completed."); continue
            elif key == 'o':
                self._do_auto(2, "Planting load", "Load completed."); continue
            elif key == 'i':
                self._do_auto(3, "Planting origin", "Origin completed."); continue
            elif key == 'l':
                self._do_auto(4, "Planting reload", "Reload completed."); continue
            elif key == 'u':
                self._do_auto(5, "Driller ...", "Driller completed."); continue

            if key == 'n':
                self.gripper_man_pub.publish(Int32(data=1)); self.add_status("Gripper move: Occupied")
            elif key == 'm':
                self.gripper_man_pub.publish(Int32(data=0)); self.add_status("Gripper move: Non_Occupied")
            elif key == 't':
                self.cartesian_man_pub.publish(Int32(data=1)); self.add_status("Cartesian move x153 203 : test")
            elif key == 'v':
                self.cartesian_man_pub.publish(Int32(data=0)); self.add_status("Anti_Vibration")
            elif key == 'k':
                self.camera_pub.publish(Int32(data=1)); self.add_status("Camera move: +")
            elif key == 'j':
                self.camera_pub.publish(Int32(data=0)); self.add_status("Camera move: -")

            if key in ['w', 'a', 's', 'd', 'c', 'r', 'q', 'e', 'f']:
                self.calculate_and_apply_velocity(key)

def main():
    rclpy.init()
    node = KeyboardVelocityPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # ส่งคำสั่งหยุดก่อนปิด node
        node.current_direction = Twist()
        node.current_velocity = 0.0
        node._publish_hold()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
