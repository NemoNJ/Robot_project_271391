#include <Arduino.h>
// #include <micro_ros_arduino.h>
#include <AccelStepper.h>
#include <micro_ros_platformio.h>
#include <ESP32Servo.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)


// -------------------- CONFIG --------------------
#define motorInterfaceType 1   // DRIVER mode (STEP + DIR)
// //---driller motor---
#define MOTOR_1  16
#define MOTOR_2  17
// --- Stepper 1 (153)---
#define stepPin1 32
#define dirPin1 33
#define enPin1 12

// --- Stepper 2 (203)---
#define stepPin2 26
#define dirPin2 27
#define enPin2 25

// --- Lead screw / motor settings ---
const int motorStepsPerRev = 200;   // step ต่อรอบ
const int microstepping = 4;        // microstep
const float leadScrewPitch = 8.0;   // mm per revolution

const float stepsPerMM = (motorStepsPerRev * microstepping) / leadScrewPitch;
double cartesian[] = {0,153,140,180}; //ระยะที่ต้องการให้เดินทาง
// -------------------- STEPPIERS --------------------
AccelStepper stepper1(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2(motorInterfaceType, stepPin2, dirPin2);

// -------------------- FUNCTIONS --------------------
void moveMM(AccelStepper &stepper, float distance_mm) {
  long targetSteps = distance_mm * stepsPerMM;
  stepper.moveTo(targetSteps);
  stepper.runToPosition();
  //  while(stepper.distanceToGo() != 0) {
  //       stepper.run();  // non-blocking loop
  //   }
}
// -------------------- CONFIG --------------------
const bool STEPPER1_DIR_INVERT = false; // ปรับตาม wiring จริงของคุณ
const bool STEPPER2_DIR_INVERT = true;  // ตามโค้ดเดิมของคุณ

inline void applyPinPolarity() {
  // EN=active LOW (ตัวที่สามเป็น true ตามของเดิม)
  stepper1.setPinsInverted(STEPPER1_DIR_INVERT, false, true);
  stepper2.setPinsInverted(STEPPER2_DIR_INVERT, false, true);
}


Servo myservo[4]; // Create an array of Servo objects
int servo_last_deg[4] = {0,0,0,0};

inline void servoWriteTracked(int idx, int deg){
  servo_last_deg[idx] = deg;
  myservo[idx].write(deg);
}
//servopin{load_plant,gripper,change end effector,camera}
int pos_degree[4] = {9,56,35,30};
int servoPin[4] = {15,23,22,2};
int change_eff_pos[2] = {0,97};
int pos = 0;
int last_msgdata = 0;
bool servoActionCompleted = true;
int lastAutoPlantValue = 0;
//auto plant status
bool check_plant = false;
int plant_count = 1;
bool back_to_zero = false;//servo load reset flag
// ROS 2 entities
rcl_publisher_t status_publisher;
rcl_subscription_t auto_plant_subscriber;
std_msgs__msg__Int32 status_msg;
std_msgs__msg__Int32 auto_plant_msg;
rcl_subscription_t move_camera_subscriber;
std_msgs__msg__Int32 move_camera_msg;
rcl_subscription_t move_gripper_subscriber;
std_msgs__msg__Int32 move_gripper_msg;
rcl_subscription_t move_cartesian_subscriber;
std_msgs__msg__Int32 move_cartesian_msg;
const float CARTESIAN_JOG_MM = 10.0f;

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

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void flashLED(unsigned int);
struct timespec getTime();
void publishData();
void moveServos();
void move_steppers();
void gripper_open_close();
void cover_the_soil();
void plantStep_funtion();
void loadServos_function();
void timer_callback(rcl_timer_t *, int64_t);
void controlCallback(rcl_timer_t *timer, int64_t last_call_time);
void move_camera_callback(const void * msgin);
void move_gripper_callback(const void * msgin);
void move_cartesian_callback(const void * msgin);
void move_camera(int mode);
void mode_load_system(int mode);
void move_steppers_01();
void move_steppers_02();
void move_steppers_03();
void move_driller();
void anti_vibration(int servo_index);
void goHomeZero(AccelStepper &s1, AccelStepper &s2, long tol_steps = 1);
void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait until the serial is connected
  Serial.println("Starting ESP32 Servo Controller");
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  set_microros_serial_transports(Serial);
  Serial.println("MicroROS transports set");
  myservo[0].attach(servoPin[0], 535, 2400);
  myservo[1].attach(servoPin[1], 400, 2400);
  myservo[2].attach(servoPin[2], 400, 2400);
  myservo[3].attach(servoPin[3], 527, 2430);
  myservo[1].write(pos_degree[1]);
  myservo[0].write(0);
  myservo[2].write(change_eff_pos[1]);
  servoWriteTracked(2, change_eff_pos[1]);
  anti_vibration(2);
  // Allow allocation of all timers

  // Stepper 1
  stepper1.setEnablePin(enPin1);
  // stepper1.setPinsInverted(false, false, true); // invert EN (active LOW)
  stepper1.setMaxSpeed(7200);
  stepper1.setAcceleration(5200);
  stepper1.setCurrentPosition(0);
  stepper1.disableOutputs();

  // Stepper 2
  stepper2.setEnablePin(enPin2);
  // stepper2.setPinsInverted(true, false,true); // invert EN (active LOW)
  stepper2.setMaxSpeed(7400);
  stepper2.setAcceleration(5500);
  stepper2.setCurrentPosition(0);
  stepper2.disableOutputs();
  applyPinPolarity();
}
void loop() {
  Serial.begin(115200);
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 1000) {
    Serial.printf("Current state: %d, Servo action completed: %s\n", 
                 state, 
                 servoActionCompleted ? "true" : "false");
    last_debug = millis();
  }
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroyEntities();
      }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}
void move_gripper_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  Serial.printf("Received move_gripper message: %d\n", msg->data);

  if (msg->data == 1) {
    // ตัวอย่าง: เปิด gripper
    myservo[1].write(pos_degree[1]);
    Serial.println("Gripper: OPEN");
  } else {
    // ปิด gripper
    myservo[1].write(pos_degree[0]);    
    Serial.println("Gripper: CLOSE");
  }
}
// >>> ADD: callback สำหรับ /move_cartesian
void move_cartesian_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *) msgin;
  if (!servoActionCompleted) {
    Serial.println("[/move_cartesian] Busy (servoActionCompleted=false), ignored");
    return;
  }
  int cmd = msg->data;
  bool test_cartesian = false;
  // ตีความ: 1 = +10mm, 0 = -10mm, อื่นๆ = ไม่ทำ
  // float delta_mm = 0.0f;
  if (cmd == 1) {
    // delta_mm = CARTESIAN_JOG_MM;      // +10
    test_cartesian = true;
  } else if (cmd == 0) {
    // delta_mm = -CARTESIAN_JOG_MM;     // -10
     servoWriteTracked(2, change_eff_pos[1]);
     anti_vibration(2);
  } else {
    Serial.printf("[/move_cartesian] Unknown cmd=%d (ignored)\n", cmd);
    return;
  }

  // long delta_steps = (long) lroundf(delta_mm * stepsPerMM);

  // Serial.printf("[/move_cartesian] cmd=%d, delta_mm=%.2f, delta_steps=%ld\n",
  //               cmd, delta_mm, delta_steps);

  // เตรียมโพลาริตี้และเปิดเอาต์พุตเฉพาะตอนขยับ
  applyPinPolarity();
  stepper1.enableOutputs();
  stepper2.enableOutputs();
  // เคลื่อนแบบ 'สัมพัทธ์' จากตำแหน่งปัจจุบัน
  // stepper1.move(delta_steps);
  // stepper1.runToPosition();
  if(test_cartesian){
    moveMM(stepper1,20.0);
    delay(1000);
    moveMM(stepper1,0.0);
    delay(1000);
    moveMM(stepper2,20.0);
    delay(1000);
    moveMM(stepper2,0.0);
  }
  stepper1.disableOutputs();
  stepper2.disableOutputs();
}
int mode = 0;
int last_mode = 0;
void move_camera_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  Serial.printf("Received move_camera message: %d\n", msg->data);
  if (mode < 0) mode = 0;
  if (mode > 2) mode = 2;
  if (msg->data == 1) {
    mode += 1;   // หมุนไปข้างหน้า
  } else {
    mode -= 1;   // หมุนถอยหลัง
  }
  if (mode < 0) mode = 0;
  if (mode > 2) mode = 2;
  move_camera(mode);
  last_mode = mode;  // อัปเดตสถานะล่าสุด
}
void move_camera(int mode){
    if(mode == 0){
      myservo[3].write(180); 
    }else if(mode == 1){
      myservo[3].write(135); 
    }else{
      myservo[3].write(90); 
    }
}

void auto_plant_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  
  Serial.printf("Received auto_plant message: %d\n", msg->data);
  
  if (msg->data == 1) {
    if (servoActionCompleted) {
      servoActionCompleted = false;
      move_driller();
      move_steppers_01();
      cover_the_soil();
      servoActionCompleted = true;
      last_msgdata = 1;
    } else {
      Serial.println("Servo is busy, skipping this command");
    }
  }else if(msg->data == 2){
    if (servoActionCompleted) {
      servoActionCompleted = false;
      if(back_to_zero == false){
          loadServos_function();
          move_steppers_02();
      } 
      servoActionCompleted = true;
      last_msgdata = 2;
    } else {
      Serial.println("Servo is busy, skipping this command");
    }
  }else if(msg->data == 3){   
    if (servoActionCompleted) {
      servoActionCompleted = false;
      move_steppers_03();
      servoActionCompleted = true;
      last_msgdata = 3;
    } else {
      Serial.println("Servo is busy, skipping this command");
    }
  }else if(msg->data == 4){
    if (servoActionCompleted) {
      servoActionCompleted = false;
      if(back_to_zero){
          loadServos_function();
          back_to_zero = false;
      }
      servoActionCompleted = true;
      last_msgdata = 4;
    }  else {
      Serial.println("Servo is busy, skipping this command");
    }
  }else if(msg->data == 5){
    if (servoActionCompleted) {
      servoActionCompleted = false;
      // move_driller();
      servoActionCompleted = true;
      last_msgdata = 5;
    }  else {
      Serial.println("Servo is busy, skipping this command");
    }
  }
  lastAutoPlantValue = msg->data;
}

//-----------------------------------------plant function start--------------------------------
double offset[10] = {0,0.11,0.12,0.12,0.12,0.22,0.18,0.11,0.1,0.1};
double number_seed[2][5] = {
  {0+offset[0],1+offset[1],2+offset[2],3+offset[3],4+offset[4]},
  {0.5+offset[5],1.5+offset[6],2.5+offset[7],3.5+offset[8],4.5+offset[9]}
};

int load_index = 0;
double pos_seed = 36.0;
int mode_load = 0;

void loadServos_function() {
  myservo[0].attach(servoPin[0], 570, 2450);
  mode_load_system(mode_load);
  myservo[0].detach();
  delay(500);  // กันสั่นเล็กน้อย
}

void mode_load_system(int mode_load) {
  if(load_index == 0) myservo[0].attach(servoPin[0], 535, 2400);
  else myservo[0].attach(servoPin[0], 528, 2480);
  if(load_index == 1 && mode_load == 0) myservo[0].attach(servoPin[0],570, 2380);
  if(load_index == 2 && mode_load == 0) myservo[0].attach(servoPin[0],570, 2430);
  if(load_index == 3) myservo[0].attach(servoPin[0], 580, 2480);
  if(load_index == 4) myservo[0].attach(servoPin[0], 620, 2480);
  if (load_index < 5) {
    double start_pos = (load_index == 0) ? 0 : number_seed[mode_load][load_index - 1] * pos_seed;
    double end_pos   = number_seed[mode_load][load_index] * pos_seed;
    if (end_pos > 180) end_pos = 180;
    for (double pos = start_pos; pos <= end_pos; pos += 5) {
      myservo[0].write(pos);
      delay(60);
    }
    load_index += 1;
  } else {
    double start_pos = number_seed[mode_load][4] * pos_seed;
    double end_pos;

    if (mode_load == 0) {
      // จบโหมด 0: 144 -> 18 แล้วไปต่อโหมด 1
      end_pos = 18;
      for (double pos = start_pos; pos >= end_pos; pos -= 2) {
        myservo[0].write(pos);
        delay(60);
      }
      load_index = 1;    // ต่อ 18 -> 54 ในคำสั่ง '1' ครั้งถัดไป
      mode_load  = 1;
    } else {
      // จบโหมด 1: ต้องรอ '0' เพื่อยืนยันกลับศูนย์
      if (!back_to_zero) {
        myservo[0].attach(servoPin[0], 535, 2400);
        back_to_zero = true;           // ตั้งธงรออนุญาต
        // ไม่ขยับ 162->0 ในรอบนี้ และไม่สลับโหมด/รีเซ็ต index
        return;
      } else {
        // ได้รับ '0' แล้ว: กลับ 162 -> 0
        end_pos = 0;
        for (double pos = start_pos; pos >= end_pos; pos -= 2) {
          myservo[0].write(pos);
          delay(40);
        }
        load_index    = 0;             // เริ่มรอบโหมด 0 ใหม่ (0 -> 36)
        mode_load     = 0;
        back_to_zero  = false;         // เคลียร์ธงหลังกลับศูนย์
      }
    }
    ::mode_load = mode_load;
  }
}
void anti_vibration(int servo_index = 2){
  myservo[servo_index].detach();
  delay(3000);  // กันสั่นตอนปล่อย
  if(servo_index == 0) myservo[servo_index].attach(servoPin[servo_index], 570, 2450);
  else myservo[servo_index].attach(servoPin[servo_index], 400, 2400);
  myservo[servo_index].write(servo_last_deg[servo_index]);
  delay(150); // กันกระตุกเล็กน้อย
}
void cover_the_soil(){
  applyPinPolarity();
  stepper1.enableOutputs();
  stepper2.enableOutputs();
  // delay(1000);//ลดเวลา
  myservo[1].write(pos_degree[1]);
  moveMM(stepper1,36.0);
  // moveMM(stepper2,174.0);
  // delay(1000);
  // moveMM(stepper2,155.0);
  myservo[1].write(pos_degree[0]);
  moveMM(stepper2,170.0);
  delay(300);
  for(int i = 0;i < 2;i++) gripper_open_close();
  //ป้องกันชนขอบ
  moveMM(stepper1,30.0);
  moveMM(stepper2,152.0);
  //---------
  moveMM(stepper1,7.0);
  moveMM(stepper2,100.0);
  stepper1.disableOutputs();
  stepper2.disableOutputs();
}
void gripper_open_close(){
   for (double pos = pos_degree[0]; pos <= 40; pos += 1) {
        myservo[1].write(pos);
        delay(10);
    }
    // delay(1000);
    for (pos = 40; pos >= pos_degree[0]; pos -= 1) {
        myservo[1].write(pos);
        delay(10);
    }
}
//-----------------------------------------plant function end--------------------------------
void goHomeZero(AccelStepper &s1, AccelStepper &s2, long tol_steps) {
  applyPinPolarity();
  s1.enableOutputs();
  s2.enableOutputs();

  long c1 = s1.currentPosition();
  long c2 = s2.currentPosition();

  // ถ้าอยู่ใกล้ศูนย์มาก ๆ ให้บังคับเขย่าออกก่อน 2 สเต็ป
  if (labs(c1) <= tol_steps) {
    s1.move( (c1 == 0) ? 2 : (c1 > 0 ? 2 : -2) );
    s1.runToPosition();
  }
  if (labs(c2) <= tol_steps) {
    s2.move( (c2 == 0) ? 2 : (c2 > 0 ? 2 : -2) );
    s2.runToPosition();
  }

  // กลับไป 0 อย่างเป็นทางการ
  moveMM(stepper2,35.0);
  myservo[2].write(change_eff_pos[1]);//หัวขุดยกลง
  servoWriteTracked(2, change_eff_pos[1]);
  s2.moveTo(0);
  s2.runToPosition();
  s1.moveTo(0);
  s1.runToPosition();
  anti_vibration(2);
  s1.disableOutputs();
  s2.disableOutputs();
}


//-----------------------------------------test function start--------------------------------
//test servo function 
void moveServos() {
  for (pos = 0; pos <= 180; pos += 1) {
    myservo[0].write(pos);
    myservo[1].write(pos);
    delay(15);
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    myservo[0].write(pos);
    myservo[1].write(pos);
    delay(15);
  }
}
void move_steppers_01() {
  // เปิดมอเตอร์ทั้งสองตัว ไปตำแหน่งปลูกต้นกล้า
  applyPinPolarity();
  stepper1.enableOutputs();
  stepper2.enableOutputs();
  moveMM(stepper1,36.0);
  myservo[2].write(change_eff_pos[0]);//ยก servo หัวขุดขึ้น
  // servoWriteTracked(2, 85);
  // anti_vibration(2);
  moveMM(stepper2,166.0);
  // servoWriteTracked(2, 0);
  // anti_vibration(2);
  myservo[1].write(pos_degree[0]);
  stepper1.disableOutputs();
  stepper2.disableOutputs();
}
void move_steppers_02() {
  // เปิดมอเตอร์ทั้งสองตัว ไปตำแหน่งโหลดต้นกล้า
  applyPinPolarity();
  stepper1.enableOutputs();
  stepper2.enableOutputs();
  // delay(1000); //ลดเวลา
  // stepper2.setPinsInverted(false, false, true);
  // myservo[1].write(0);
  moveMM(stepper2,66.0);
  myservo[2].write(change_eff_pos[0]);
  if(mode_load == 0){
     myservo[1].write(pos_degree[2]);
    moveMM(stepper1,135.0);
  } 
  else{
    myservo[1].write(pos_degree[3]);
    moveMM(stepper1,148.0);
  } 
  delay(500);//delay(1000);//ลดเวลา
  myservo[1].write(pos_degree[1]);
  delay(300);
  moveMM(stepper2,32.0);//ยกออก จากที่วางต้นกล้า
  stepper1.disableOutputs();
  stepper2.disableOutputs();
}
void move_steppers_03() {
  //back to zero position
   goHomeZero(stepper1, stepper2, 1); // tol = 1 step
}
void move_driller(){
  applyPinPolarity();
  stepper1.enableOutputs();
  stepper2.enableOutputs();
  moveMM(stepper1,96.0);//driller position
  moveMM(stepper2,35.0);
  myservo[2].write(change_eff_pos[1]);
  // servoWriteTracked(2, change_eff_pos[1]);
  // anti_vibration(2);
  delay(500); 
  //driller
    stepper2.setMaxSpeed(2450);
    stepper2.setAcceleration(2550);
    digitalWrite(MOTOR_1, LOW);
    digitalWrite(MOTOR_2,HIGH);
    moveMM(stepper2,140.0);
    // delay(300); 
    stepper2.setMaxSpeed(7400);
    stepper2.setAcceleration(5500);
    moveMM(stepper2,35.0);
    digitalWrite(MOTOR_1, LOW);
    digitalWrite(MOTOR_2,LOW);
  // myservo[2].write(90);
  stepper1.disableOutputs();
  stepper2.disableOutputs();
}

//-----------------------------------------test function end--------------------------------

bool createEntities() {
  allocator = rcl_get_default_allocator();
  
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 10);
  
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  RCCHECK(rclc_node_init_default(&node, "esp32_servo_controller", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(&status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/servo_status"));
  RCCHECK(rclc_subscription_init_default(&auto_plant_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/auto_plant"));
  RCCHECK(rclc_subscription_init_default(&move_cartesian_subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),"/move_cartesian"));


  const unsigned int control_timeout = 70;
  RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(control_timeout), controlCallback));
  
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor,&move_cartesian_subscriber,&move_cartesian_msg,&move_cartesian_callback,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &auto_plant_subscriber, &auto_plant_msg, &auto_plant_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  RCCHECK(rclc_subscription_init_default(&move_camera_subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),"/move_camera"));
  RCCHECK(rclc_executor_add_subscription(&executor,&move_camera_subscriber,&move_camera_msg,&move_camera_callback,ON_NEW_DATA));
  // subscribe /move_gripper
  RCCHECK(rclc_subscription_init_default(&move_gripper_subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),"/move_gripper"));
  RCCHECK(rclc_executor_add_subscription(&executor,&move_gripper_subscriber,&move_gripper_msg,&move_gripper_callback,ON_NEW_DATA));
  syncTime();
  
  return true;
}

bool destroyEntities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_subscription_fini(&move_gripper_subscriber, &node);
  rcl_subscription_fini(&move_camera_subscriber, &node);
  rcl_subscription_fini(&move_cartesian_subscriber, &node);
  rcl_publisher_fini(&status_publisher, &node);
  rcl_subscription_fini(&auto_plant_subscriber, &node);
  rcl_node_fini(&node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
  
  return true;
}

void publishData() {
  status_msg.data = servoActionCompleted ? 1 : 0; // 1 = ready, 0 = busy
  rcl_publish(&status_publisher, &status_msg, NULL);
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    publishData();
  }
}

void syncTime() {
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  time_offset = ros_time_ms - now;
}

struct timespec getTime() {
  struct timespec tp = {0};
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}

void rclErrorLoop() {
  while (true) {
    flashLED(3);
  }
}

void flashLED(unsigned int n_times) {
  delay(1000);
}