#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#define motorRF 1
#define motorRB 2
#define motorLF 3
#define motorLB 4

// ROS2 node และ executor
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t vel_sub;
rcl_subscription_t ang_sub;
float speed = 0.0;
float angular_speed = 0.0;
// Message ตัวแปร
std_msgs__msg__Float32 vel_msg;
std_msgs__msg__Float32 ang_msg;

// Callback สำหรับความเร็วเชิงเส้น
void velocity_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  Serial.print("[Velocity] Linear Velocity: ");
  Serial.println(msg->data, 2);
  speed = msg->data;
}

// Callback สำหรับความเร็วเชิงมุม
void angular_velocity_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  Serial.print("[Angular] Angular Velocity: ");
  Serial.println(msg->data, 2);
  angular_speed = msg->data;
}

void setup()
{
  Serial.begin(9600);
  delay(2000);
  pinMode(motorRF, OUTPUT);
  pinMode(motorRB, OUTPUT);
  pinMode(motorLF, OUTPUT);
  pinMode(motorLB, OUTPUT);

  // เชื่อมต่อแบบ Serial
  set_microros_serial_transports(Serial);

  // สร้าง allocator & support
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("Failed to init support");
    return;
  }

  // สร้าง Node
  if (rclc_node_init_default(&node, "esp32_dual_subscriber_node", "", &support) != RCL_RET_OK) {
    Serial.println("Failed to create node");
    return;
  }

  // สร้าง Subscriber สำหรับ /velocity
  if (rclc_subscription_init_default(
        &vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "velocity") != RCL_RET_OK)
  {
    Serial.println("Failed to create velocity subscriber");
    return;
  }

  // สร้าง Subscriber สำหรับ /angular_velocity
  if (rclc_subscription_init_default(
        &ang_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "angular_velocity") != RCL_RET_OK)
  {
    Serial.println("Failed to create angular velocity subscriber");
    return;
  }

  // สร้าง executor และเพิ่ม callback
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) {
    Serial.println("Failed to init executor");
    return;
  }

  rclc_executor_add_subscription(&executor, &vel_sub, &vel_msg, &velocity_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &ang_sub, &ang_msg, &angular_velocity_callback, ON_NEW_DATA);

  Serial.println("micro-ROS dual subscriber ready.");
}
void motor_control(){
      if(speed != 0.0 && angular_speed != 0.0){
        speed = 0.0;
        angular_speed = 0.0;
      }
      if(speed >= 0){
        analogWrite(motorRF, speed);
        delay(1);
        analogWrite(motorLF, speed);
        delay(1);
        analogWrite(motorRB, 0.0);
        delay(1);
        analogWrite(motorLB, 0.0);
        delay(1);
      }else{
        analogWrite(motorRF, 0.0);
        delay(1);
        analogWrite(motorLF, 0.0);
        delay(1);
        analogWrite(motorRB, speed);
        delay(1);
        analogWrite(motorLB, speed);
        delay(1);
      }
      if(angular_speed >= 0){
        analogWrite(motorRF, 0.0);
        delay(1);
        analogWrite(motorLF, speed);
        delay(1);
        analogWrite(motorRB, speed);
        delay(1);
        analogWrite(motorLB, 0.0);
        delay(1);
      }else{
        analogWrite(motorRF, speed);
        delay(1);
        analogWrite(motorLF, 0.0);
        delay(1);
        analogWrite(motorRB, 0.0);
        delay(1);
        analogWrite(motorLB, speed);
        delay(1);
      }


}
void loop()
{
   
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  motor_control()
  delay(10);
}