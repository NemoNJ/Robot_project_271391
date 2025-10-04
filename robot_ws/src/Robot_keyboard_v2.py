#!/usr/bin/env python3
import sys
import tty
import termios
import select
import shutil  # << ใช้หาขนาดหน้าจอ
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

# ถ้าต้องการ QoS เฉพาะ ให้เปิดคอมเมนต์บรรทัดด้านล่าง
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class KeyboardVelocityPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_velocity_publisher_node')

        # ---- Publishers ----
        self.linear_pub = self.create_publisher(Float32, '/velocity', 10)
        self.angular_pub = self.create_publisher(Float32, '/angular_velocity', 10)
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
        self.current_linear = 0.0
        self.current_angular = 0.0

        # ค่าที่แสดงบนหน้าจอ
        self.last_linear = 0.0
        self.last_angular = 0.0
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
        sys.stdout.write(f"Published: Linear = {self.last_linear:.1f}, Angular = {self.last_angular:.1f}\033[K")

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
        self.linear_pub.publish(Float32(data=self.current_linear))
        self.angular_pub.publish(Float32(data=self.current_angular))
        self.last_linear = self.current_linear
        self.last_angular = self.current_angular

    def calculate_and_apply_velocity(self, key):
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

        if key in ['s'] and self.prev_key in ['a', 'd', 'w', 'e', 'q']:
            self.linear_level = 0
        self.prev_key = key

        raw_linear = (self.linear_level * self.step / 2) if key in ['r', 'f'] else (self.linear_level * self.step)
        linear_speed = float(min(raw_linear, self.max_speed)) if raw_linear > 0 else float(max(raw_linear, -self.max_speed))

        raw_angular = (self.angular_level * self.step_turn * 1.8) if key in ['q', 'e'] else (self.angular_level * self.step_turn * 2)
        angular_speed = float(min(raw_angular, self.max_speed)) if raw_angular > 0 else float(max(raw_angular, -self.max_speed))

        self.current_linear = linear_speed
        self.current_angular = angular_speed
        self.add_status(f"[KEY:{key}] Linear: {linear_speed:.1f} | Angular: {angular_speed:.1f}")
        return True

    def _zero_and_log(self, msg: str):
        self.current_linear = 0.0
        self.current_angular = 0.0
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
        node.current_linear = 0.0
        node.current_angular = 0.0
        node._publish_hold()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
