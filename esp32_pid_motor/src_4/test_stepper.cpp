#include <AccelStepper.h>

// -------------------- CONFIG --------------------
#define motorInterfaceType 1   // DRIVER mode (STEP + DIR)

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

// -------------------- STEPPIERS --------------------
AccelStepper stepper1(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2(motorInterfaceType, stepPin2, dirPin2);

// -------------------- FUNCTIONS --------------------
void moveMM(AccelStepper &stepper, float distance_mm) {
  long targetSteps = distance_mm * stepsPerMM;
  stepper.moveTo(targetSteps);
  stepper.runToPosition();
}

void move_steppers() {
  // เปิดมอเตอร์ทั้งสองตัว
  stepper1.enableOutputs();
  stepper2.enableOutputs();
  // เดินหน้า
  moveMM(stepper1, 0.0);  // Stepper 1
  moveMM(stepper2, 70.0);  // Stepper 2
  delay(5000);

  // กลับ
  moveMM(stepper1, 70.0);
  moveMM(stepper2, 0.0);
  delay(5000);
  stepper1.disableOutputs();
  stepper2.disableOutputs();
}
// void test_heat_stepper(){
//   for(int i = 1;i <= 3;i++){move_steppers();};
// }

// -------------------- SETUP --------------------
void setup() {
  // Stepper 1
  stepper1.setEnablePin(enPin1);
  stepper1.setPinsInverted(false, false, true); // invert EN (active LOW)
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);
  stepper1.setCurrentPosition(0);
  stepper1.disableOutputs();

  //Stepper 2
  stepper2.setEnablePin(enPin2);
  stepper2.setPinsInverted(false, false,true); // invert EN (active LOW)
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);
  stepper2.setCurrentPosition(0);
  stepper2.disableOutputs();
}

// -------------------- LOOP --------------------
bool check = true;
//เราจะตรวจว่า disabled ช่วยลดความร้อนได้ไหม
void loop() {
  // if(check == true){
  //   test_heat_stepper();
  //   check = false;
  // } 
   // ปิดมอเตอร์ทั้งสองตัว
   move_steppers();
}

