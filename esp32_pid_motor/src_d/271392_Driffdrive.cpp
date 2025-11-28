#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <vector>
#include <cmath>
#include <utility>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#include <config.h>
#include <motor.h>
#include <PIDF.h>
// #include <ESP32Encoder.h>  // << ลบการใช้งาน encoder ออก
#include <Utilize.h>

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            rclErrorLoop();          \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)


// ##############################################


// ROS variables

// ---- Publisher ----
rcl_publisher_t status_publisher;
std_msgs__msg__Int32 status_msg;

// ---- Subscribers (ใหม่ ให้สอดคล้องกับ keyboard node) ----
rcl_subscription_t robot_velocity_subscriber;      // /robot_velocity (Float32)
rcl_subscription_t robot_direction_subscriber;     // /robot_direction (Twist)

std_msgs__msg__Float32 robot_velocity_msg;
geometry_msgs__msg__Twist robot_direction_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long current_time = 0;
static unsigned long last_pub = 0;
static int disconnect_count = 0;

// Motor Pins

#define MOTOR_R_IN1_1 18  // ล้อขวาหน้า
#define MOTOR_R_IN2_1 19
#define MOTOR_R_IN1_2 22  // ล้อขวาหลัง
#define MOTOR_R_IN2_2 23

#define MOTOR_L_IN1_1 32  // ล้อซ้ายหน้า
#define MOTOR_L_IN2_1 33
#define MOTOR_L_IN1_2 26  // ล้อซ้ายหลัง
#define MOTOR_L_IN2_2 27


enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//------------------------------ < Function Prototype > ------------------------------//
  
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void flashLED(unsigned int);
struct timespec getTime();

void publishData();
void set_motor_speed(int, int, float);

// ฟังก์ชันใคำนวณความเร็วล้อจาก /robot_direction และ /robot_velocity
void update_motors_from_cmd();

//------------------------------ < Main > -------------------------------------//

void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
}
  
void loop()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    switch (state)
    {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1000,
        state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT)
      {
          destroyEntities();
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if (state == AGENT_CONNECTED)
      {
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(300));
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      disconnect_count = 0;
      state = WAITING_AGENT;
      break;

    default:
      break;
    }
}

//------------------------------ < Callbacks > -------------------------------------//

// callback เมื่อได้รับ /robot_velocity (Float32)
void robot_velocity_callback(const void * msgin)
{
    robot_velocity_msg.data = ((const std_msgs__msg__Float32*)msgin)->data;
    update_motors_from_cmd();
}

// callback เมื่อได้รับ /robot_direction (Twist)
void robot_direction_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist*)msgin;
    robot_direction_msg = *msg;  // copy struct ตรง ๆ ได้
    update_motors_from_cmd();
}

// timer callback (ใช้สำหรับ publishData)
void timer_callback(rcl_timer_t *, int64_t)
{
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        publishData();
    }
}

// ฟังก์ชันหลักสำหรับคำนวณความเร็วล้อจาก direction + velocity
void update_motors_from_cmd()
{
    float dir_linear  = robot_direction_msg.linear.x;  // +1, -1, 0
    float dir_angular = robot_direction_msg.angular.z; // +1, -1, 0
    float speed       = robot_velocity_msg.data;       // ≥ 0, จาก keyboard

    float v = 0.0f;
    float omega = 0.0f;

    // เงื่อนไข: ถ้า linear.x != 0 -> angular.z ต้อง = 0
    // ถ้า angular.z != 0 -> linear.x ต้อง = 0
    // ถ้าทั้งคู่เป็น 0 -> หยุด
    if (dir_linear != 0.0f)
    {
        v = dir_linear * speed;
        omega = 0.0f;
    }
    else if (dir_angular != 0.0f)
    {
        v = 0.0f;
        omega = dir_angular * speed;
    }
    else
    {
        v = 0.0f;
        omega = 0.0f;
    }

    // ใช้สูตรเดิมเหมือน angular_velocity_callback:
    // left_speed  = v - omega
    // right_speed = v + omega
    float left_speed  = v - omega;
    float right_speed = v + omega;

    // คุมมอเตอร์ทั้ง 4 ล้อ
    set_motor_speed(MOTOR_L_IN1_1, MOTOR_L_IN2_1, left_speed);
    set_motor_speed(MOTOR_L_IN1_2, MOTOR_L_IN2_2, left_speed);

    // ด้านขวายังคงคูณ (-1) แบบเดิม
    set_motor_speed(MOTOR_R_IN1_1, MOTOR_R_IN2_1, (-1) * right_speed);
    set_motor_speed(MOTOR_R_IN1_2, MOTOR_R_IN2_2, (-1) * right_speed);
}

//------------------------------ < ROS Entities > -------------------------------------//
  
bool createEntities()
{
    allocator = rcl_get_default_allocator();
  
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);
  
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  
    // create node
    RCCHECK(rclc_node_init_default(&node, "esp32", "", &support));
  
    // Publisher: motor_status
    RCCHECK(rclc_publisher_init_best_effort(
        &status_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/motor_status"));

    // Subscriptions
    // 1) /robot_velocity (Float32)
    RCCHECK(rclc_subscription_init_default(
        &robot_velocity_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/robot_velocity"));

    // 2) /robot_direction (Twist)
    RCCHECK(rclc_subscription_init_default(
        &robot_direction_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/robot_direction"));

    // Timer สำหรับ publishData
    const unsigned int control_timeout = 70;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));

    // Executor: มี 2 subs + 1 timer = 3 handles
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &robot_velocity_subscriber,
        &robot_velocity_msg,
        &robot_velocity_callback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &robot_direction_subscriber,
        &robot_direction_msg,
        &robot_direction_callback,
        ON_NEW_DATA));

    syncTime();
    
    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&status_publisher, &node);
    rcl_subscription_fini(&robot_velocity_subscriber, &node);
    rcl_subscription_fini(&robot_direction_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
    
    return true;
}
  
void publishData()
{
    //  status ว่ามอเตอร์ทำงานหรือไม่
    float speed = robot_velocity_msg.data;
    status_msg.data = (fabs(speed) > 0.1f) ? 1 : 0;
    rcl_publish(&status_publisher, &status_msg, NULL);
}

void syncTime()
{
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}
      
struct timespec getTime()
{
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

void rclErrorLoop()
{
    ESP.restart();
    while (true)
    {
        flashLED(3);
    }
}
  
void flashLED(unsigned int n_times)
{
    delay(1000);
}

void set_motor_speed(int in1_pin, int in2_pin, float speed) {
    if (speed > 0.0) {         // เดินหน้า
        analogWrite(in1_pin, abs(speed));
        analogWrite(in2_pin, 0.0);
    } else if (speed < 0.0) {  // ถอยหลัง
        analogWrite(in1_pin, 0.0);
        analogWrite(in2_pin, abs(speed));
    } else {                   // หยุด
        analogWrite(in1_pin, 0.0);
        analogWrite(in2_pin, 0.0);
    }
}
