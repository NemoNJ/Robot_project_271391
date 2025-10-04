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
#include <ESP32Encoder.h>  // Use ESP32Encoder library
// #include <esp32_hardware.h>
// #include <Adafruit_I2CDevice.h>
// #include <Adafruit_AS5600.h>
#include <Utilize.h>
// #include <TCA9548A.h>

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
rcl_publisher_t wheel_rpm_publisher;
geometry_msgs__msg__Twist wheel_rpm_msg;

rcl_publisher_t status_publisher;
rcl_subscription_t velocity_subscriber;
rcl_subscription_t angular_velocity_subscriber;  // เพิ่มนี้ใกล้กับ velocity_subscriber

std_msgs__msg__Int32 status_msg;
std_msgs__msg__Float32 velocity_msg;
std_msgs__msg__Float32 angular_velocity_msg;  // เพิ่มนี้ใกล้กับ velocity_msg

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
// // Motor Pins สำหรับ 4 ล้อ
// #define MOTOR_L_IN1_1 18  // ล้อซ้ายหน้า
// #define MOTOR_L_IN2_1 19
// #define MOTOR_L_IN1_2 22  // ล้อซ้ายหลัง
// #define MOTOR_L_IN2_2 23
// #define MOTOR_R_IN1_1 32  // ล้อขวาหน้า
// #define MOTOR_R_IN2_1 33
// #define MOTOR_R_IN1_2 26  // ล้อขวาหลัง
// #define MOTOR_R_IN2_2 27


#define MOTOR_R_IN1_1 18  // ล้อขวาหน้า
#define MOTOR_R_IN2_1 19
#define MOTOR_R_IN1_2 22  // ล้อขวาหลัง
#define MOTOR_R_IN2_2 23

#define MOTOR_L_IN1_1 32  // ล้อซ้ายหน้า
#define MOTOR_L_IN2_1 33
#define MOTOR_L_IN1_2 26  // ล้อซ้ายหลัง
#define MOTOR_L_IN2_2 27

#define ENCODER_R1_A 34
#define ENCODER_R1_B 35
#define ENCODER_R2_A 36
#define ENCODER_R2_B 39
#define ENCODER_L1_A 16
#define ENCODER_L1_B 4
#define ENCODER_L2_A 5
#define ENCODER_L2_B 17

// Define Encoder objects
ESP32Encoder encoder_r1;
ESP32Encoder encoder_r2;
ESP32Encoder encoder_l1;
ESP32Encoder encoder_l2;

float rpm_r1 = 0;
float rpm_r2 = 0;
float rpm_l1 = 0;
float rpm_l2 = 0;

unsigned long last_time_r1 = 0;
unsigned long last_time_r2 = 0;
unsigned long last_time_l1 = 0;
unsigned long last_time_l2 = 0;
// ----- Motor calibration (วางไว้หลังส่วนกำหนดพินมอเตอร์) -----
struct MotorCal {
  float fwd_scale;   // สเกลตอนเดินหน้า
  float rev_scale;   // สเกลตอนถอยหลัง
  bool  invert;      // ต้องกลับเฟสไหม (ล้อขวาส่วนใหญ่จะ true)
};

// ปรับตัวเลขตามที่คุณวัดจริงได้เลย
// ซ้าย
static const MotorCal CAL_LF = {0.885f,   0.985f,   false};   // Left Front  (MOTOR_L_IN1_1, MOTOR_L_IN2_1)
static const MotorCal CAL_LR = {0.917f,   0.985f,   false};   // Left Rear   (MOTOR_L_IN1_2, MOTOR_L_IN2_2)
// ขวา (กลับเฟส)
static const MotorCal CAL_RF = {0.988707f,0.988707f, true};   // Right Front (MOTOR_R_IN1_1, MOTOR_R_IN2_1)
static const MotorCal CAL_RR = {0.987807f,0.987807f, true};   // Right Rear  (MOTOR_R_IN1_2, MOTOR_R_IN2_2)

// helper เลือกคาลิเบรตจากพิน
static inline const MotorCal& cal_for(int in1_pin, int in2_pin) {
  if (in1_pin == MOTOR_L_IN1_1 && in2_pin == MOTOR_L_IN2_1) return CAL_LF;
  if (in1_pin == MOTOR_L_IN1_2 && in2_pin == MOTOR_L_IN2_2) return CAL_LR;
  if (in1_pin == MOTOR_R_IN1_1 && in2_pin == MOTOR_R_IN2_1) return CAL_RF;
  // else MOTOR_R_IN1_2 / MOTOR_R_IN2_2
  return CAL_RR;
}
enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

  //------------------------------ < Fuction Prototype > ------------------------------//
  
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void flashLED(unsigned int);
struct timespec getTime();

void publishData();
void set_motor_speed(int, int, float);
//------------------------------ < Main > -------------------------------------//

void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

      // Setup Encoder
    encoder_r1.attachHalfQuad(ENCODER_R1_A, ENCODER_R1_B);
    encoder_r2.attachHalfQuad(ENCODER_R2_A, ENCODER_R2_B);
    encoder_l1.attachHalfQuad(ENCODER_L1_A, ENCODER_L1_B);
    encoder_l2.attachHalfQuad(ENCODER_L2_A, ENCODER_L2_B);
  }
  
  void loop()
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    
    // Read RPM values from encoders
    unsigned long current_time = millis();

    rpm_r1 = (encoder_r1.getCount() / ((current_time - last_time_r1) / 1000.0)) * 60.0*(1/379.0); // 337.0 is the gear ratio
    rpm_r2 = (encoder_r2.getCount() / ((current_time - last_time_r2) / 1000.0)) * 60.0*(1/379.0);
    rpm_l1 = (-1)*(encoder_l1.getCount() / ((current_time - last_time_l1) / 1000.0)) * 60.0*(1/379.0);
    rpm_l2 = (-1)*(encoder_l2.getCount() / ((current_time - last_time_l2) / 1000.0)) * 60.0*(1/379.0);

    encoder_r1.clearCount();
    encoder_r2.clearCount();
    encoder_l1.clearCount();
    encoder_l2.clearCount();

    last_time_r1 = last_time_r2 = last_time_l1 = last_time_l2 = current_time;
    switch (state)
    {
    case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
        case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
          }
          break;
          case AGENT_CONNECTED:
          EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
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
    
//------------------------------ < Fuction > -------------------------------------//


void velocity_callback(const void * msgin) {
  velocity_msg.data = ((const std_msgs__msg__Float32*)msgin)->data; // ค่านี้ยังเป็น PWM เดิมได้

  float v = velocity_msg.data;
  set_motor_speed(MOTOR_L_IN1_1, MOTOR_L_IN2_1, v);
  set_motor_speed(MOTOR_L_IN1_2, MOTOR_L_IN2_2, v);
  set_motor_speed(MOTOR_R_IN1_1, MOTOR_R_IN2_1, v);
  set_motor_speed(MOTOR_R_IN1_2, MOTOR_R_IN2_2, v);
}

void angular_velocity_callback(const void * msgin) {
  float angular_speed = ((const std_msgs__msg__Float32*)msgin)->data;

  float left_speed  = velocity_msg.data - angular_speed;
  float right_speed = velocity_msg.data + angular_speed;

  set_motor_speed(MOTOR_L_IN1_1, MOTOR_L_IN2_1, left_speed);
  set_motor_speed(MOTOR_L_IN1_2, MOTOR_L_IN2_2, left_speed);
  set_motor_speed(MOTOR_R_IN1_1, MOTOR_R_IN2_1, right_speed);
  set_motor_speed(MOTOR_R_IN1_2, MOTOR_R_IN2_2, right_speed);
}

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

  
bool createEntities()
{
  allocator = rcl_get_default_allocator();
  
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 10);
  
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  
  // create node
  RCCHECK(rclc_node_init_default(&node, "esp32", "", &support));
  
  // Pub
  RCCHECK(rclc_publisher_init_best_effort(
    &status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/motor_status"));

  // Sub
      
  RCCHECK(rclc_subscription_init_default(
    &velocity_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/velocity"));
  
    // เพิ่ม subscription สำหรับ angular velocity
  RCCHECK(rclc_subscription_init_default(
    &angular_velocity_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/angular_velocity"));
  RCCHECK(rclc_publisher_init_best_effort(&wheel_rpm_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/wheel_rpm"));

  // create timer for actuating the motors at 50 Hz (1000/20)
  const unsigned int control_timeout = 70;
  RCCHECK(rclc_timer_init_default(
    &control_timer,
    &support,
    RCL_MS_TO_NS(control_timeout),
    controlCallback));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &velocity_subscriber,
    &velocity_msg,
    &velocity_callback,
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &angular_velocity_subscriber,
    &angular_velocity_msg,
    &angular_velocity_callback,
    ON_NEW_DATA));
    syncTime();
    
    return true;
}
  bool destroyEntities()
  {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_publisher_fini(&wheel_rpm_publisher, &node);
  rcl_publisher_fini(&status_publisher, &node);
  rcl_subscription_fini(&velocity_subscriber, &node);
  rcl_subscription_fini(&angular_velocity_subscriber, &node);
  rcl_node_fini(&node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
    
    return true;
  }
  
  void publishData()
  {
    status_msg.data = velocity_msg.data > 0.1 ? 1 : 0;
    rcl_publish(&status_publisher, &status_msg, NULL);
    geometry_msgs__msg__Twist wheel_rpm_msg;
    wheel_rpm_msg.linear.x = int(rpm_r1);
    wheel_rpm_msg.linear.y = int(rpm_r2);
    wheel_rpm_msg.linear.z = int(rpm_l1);
    wheel_rpm_msg.angular.x = int(rpm_l2);
    wheel_rpm_msg.angular.y = 0;
    wheel_rpm_msg.angular.z = 0;
    RCCHECK(rcl_publish(&wheel_rpm_publisher, &wheel_rpm_msg, NULL));
    // RCCHECK(rcl_publish(&status_publisher, &wheel_rpm_msg, NULL));
  }

  void syncTime()
  {
    // get the current time from the agent
    unsigned long now = millis();
        RCCHECK(rmw_uros_sync_session(10));
        unsigned long long ros_time_ms = rmw_uros_epoch_millis();
        // now we can find the difference between ROS time and uC time
        time_offset = ros_time_ms - now;
      }
      
struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
        tp.tv_sec = now / 1000;
        tp.tv_nsec = (now % 1000) * 1000000;
        return tp;
}

void rclErrorLoop()
{
    while (true)
    {
      flashLED(3);
    }
  }
  
void flashLED(unsigned int n_times)
{
  // for (int i = 0; i < n_times; i++)
  // {
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   delay(100);
  //   digitalWrite(LED_BUILTIN, LOW);
  //   delay(100);
  //   }
    delay(1000);
}

// ----- แทนที่ของเดิมทั้งหมด -----
static inline int clamp_pwm(float x, int maxDuty=255) {
  // แปลงเป็น duty บวก 0..maxDuty พร้อม clamp
  int d = (int)roundf(fabsf(x));
  if (d < 0) d = 0;
  if (d > maxDuty) d = maxDuty;
  return d;
}

void set_motor_speed(int in1_pin, int in2_pin, float speed) {
  const MotorCal& cal = cal_for(in1_pin, in2_pin);

  // จัดการทิศที่มอเตอร์นี้ต้องการ (ล้อขวา invert=true)
  float s = cal.invert ? -speed : speed;

  // เลือกสเกลตามทิศทาง
  float scaled = (s >= 0.0f) ? (s * cal.fwd_scale) : (s * cal.rev_scale);

  // Deadband + min duty (ช่วยชนะ static friction)
  const int MAX_DUTY = 255;       // ถ้าตั้ง LEDC อย่างอื่น ปรับตรงนี้
  const int MIN_DUTY = 40;        // ปรับตามรถคุณ
  const float DEADBAND = 5.0f;    // |scaled| ต่ำกว่านี้ให้หยุด (ถ้าคุณส่งย่าน 0..255)

  if (fabsf(scaled) <= DEADBAND) {
    analogWrite(in1_pin, 0);
    analogWrite(in2_pin, 0);
    return;
  }

  int duty = clamp_pwm(scaled, MAX_DUTY);
  if (duty > 0 && duty < MIN_DUTY) duty = MIN_DUTY;

  if (scaled >= 0.0f) {
    // เดินหน้า
    analogWrite(in1_pin, duty);
    analogWrite(in2_pin, 0);
  } else {
    // ถอยหลัง
    analogWrite(in1_pin, 0);
    analogWrite(in2_pin, duty);
  }
}
