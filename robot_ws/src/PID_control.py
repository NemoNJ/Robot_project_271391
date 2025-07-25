import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller_node')
        # ส่งความเร็วออก
        self.linear_pub = self.create_publisher(Float32, '/velocity', 10)
        self.angular_pub = self.create_publisher(Float32, '/angular_velocity', 10)
        # รับจาก keyboard_input
        self.subscription = self.create_subscription(
            String,
            '/keyboard_input',
            self.keyboard_callback,
            10
        )

        # ระดับความเร็ว -5 ถึง +5
        self.linear_level = 0
        self.angular_level = 0
        
        # ค่าความเร็วต่อระดับ
        self.step = 51

        self.prev_key = ''  # <== เพิ่มตัวแปรนี้เพื่อเก็บ key ก่อนหน้า

    def keyboard_callback(self, msg: String):
        key = msg.data.lower()
        
    # Reset angular level ถ้าเพิ่งเดินตรงแล้วจะหมุน
        if key in ['a', 'd'] and self.prev_key in ['w', 's']:
            self.angular_level = 0

    # Reset linear level ถ้าเพิ่งหมุนแล้วจะเดินตรง
        if key in ['w', 's'] and self.prev_key in ['a', 'd']:
            self.linear_level = 0
        # จัดการการเพิ่ม/ลดระดับความเร็ว
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
        else:
            return  # ไม่ใช่ key ที่ใช้ควบคุม

        self.prev_key = key  # <== บันทึก key ที่เพิ่งกดล่าสุด

        # คำนวณและ Publish ความเร็ว
        linear_speed = Float32()
        angular_speed = Float32()

        linear_speed.data = float(self.linear_level * self.step)
        angular_speed.data = float(self.angular_level * self.step)

        self.linear_pub.publish(linear_speed)
        self.angular_pub.publish(angular_speed)

        self.get_logger().info(
            f"[KEY:{key}] Linear: {linear_speed.data}  |  Angular: {angular_speed.data}"
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