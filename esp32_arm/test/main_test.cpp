#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// GPIO pins
#define motorRF 32
#define motorRB 33
#define motorLF 25
#define motorLB 26

// PWM channels
#define ch_RF 0
#define ch_RB 1
#define ch_LF 2
#define ch_LB 3

// ROS entities
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t vel_sub, ang_sub;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

std_msgs__msg__Float32 vel_msg;
std_msgs__msg__Float32 ang_msg;

float speed = 0.0;
float angular_speed = 0.0;
float maxSpeed = 255.0;
unsigned long last_cmd_time = 0;

// Agent state
enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Macros
#define RCCHECK(fn)                  \
  {                                  \
    rcl_ret_t temp_rc = fn;          \
    if ((temp_rc != RCL_RET_OK))     \
    {                                \
      while (1)                      \
      {                              \
        Serial.println("Error!");    \
        delay(1000);                 \
      }                              \
    }                                \
  }

#define EXECUTE_EVERY_N_MS(MS, X)          \
  do                                       \
  {                                        \
    static volatile int64_t init = -1;     \
    if (init == -1)                        \
    {                                      \
      init = uxr_millis();                 \
    }                                      \
    if (uxr_millis() - init > MS)          \
    {                                      \
      X;                                   \
      init = uxr_millis();                 \
    }                                      \
  } while (0)

// Callbacks
void velocity_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  speed = msg->data;
  last_cmd_time = millis();
}

void angular_velocity_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  angular_speed = msg->data;
  last_cmd_time = millis();
}

// Motor control logic
void motor_control()
{
  float linear = speed;
  float angular = angular_speed;

  float left_speed = linear + angular;
  float right_speed = linear - angular;

  left_speed = constrain(left_speed, -maxSpeed, maxSpeed);
  right_speed = constrain(right_speed, -maxSpeed, maxSpeed);

  if (millis() - last_cmd_time > 500) {
    left_speed = 0;
    right_speed = 0;
  }

  // Left motor
  if (left_speed >= 0) {
    ledcWrite(ch_LF, abs(left_speed));
    ledcWrite(ch_LB, 0);
  } else {
    ledcWrite(ch_LF, 0);
    ledcWrite(ch_LB, abs(left_speed));
  }

  // Right motor
  if (right_speed >= 0) {
    ledcWrite(ch_RF, abs(right_speed));
    ledcWrite(ch_RB, 0);
  } else {
    ledcWrite(ch_RF, 0);
    ledcWrite(ch_RB, abs(right_speed));
  }
}

// Create entities
bool create_entities()
{
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 10));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  RCCHECK(rclc_node_init_default(&node, "esp32_motors_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "velocity"));

  RCCHECK(rclc_subscription_init_default(
      &ang_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "angular_velocity"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  RCCHECK(rclc_executor_add_subscription(
      &executor, &vel_sub, &vel_msg, &velocity_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
      &executor, &ang_sub, &ang_msg, &angular_velocity_callback, ON_NEW_DATA));

  return true;
}

// Destroy entities
bool destroy_entities()
{
  rcl_subscription_fini(&vel_sub, &node);
  rcl_subscription_fini(&ang_sub, &node);
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
  rcl_init_options_fini(&init_options);
  return true;
}

// Emergency stop
void full_stop()
{
  ledcWrite(ch_LF, 0);
  ledcWrite(ch_LB, 0);
  ledcWrite(ch_RF, 0);
  ledcWrite(ch_RB, 0);
}

// Setup
void setup()
{
  Serial.begin(115200);
  delay(2000);

  // PWM Setup
  ledcAttachPin(motorRF, ch_RF);
  ledcAttachPin(motorRB, ch_RB);
  ledcAttachPin(motorLF, ch_LF);
  ledcAttachPin(motorLB, ch_LB);

  ledcSetup(ch_RF, 5000, 8);
  ledcSetup(ch_RB, 5000, 8);
  ledcSetup(ch_LF, 5000, 8);
  ledcSetup(ch_LB, 5000, 8);

  set_microros_serial_transports(Serial);
  state = WAITING_AGENT;
}

// Main loop
void loop()
{
  switch (state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500,
      state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
    );
    break;

  case AGENT_AVAILABLE:
    state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      destroy_entities();
    }
    break;

  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200,
      state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
    );
    if (state == AGENT_CONNECTED)
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      motor_control();
    }
    break;

  case AGENT_DISCONNECTED:
    full_stop();
    destroy_entities();
    state = WAITING_AGENT;
    break;
  }
}
