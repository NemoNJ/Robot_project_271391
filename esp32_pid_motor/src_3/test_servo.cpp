#include <ESP32Servo.h>
#include <Arduino.h>

Servo myservo[3];
int servoPin = 15;
double offset[10] = {0,0.11,0.12,0.12,0.12,0.22,0.18,0.11,0.1,0.1};
double number_seed[2][5] = {
  {0+offset[0],1+offset[1],2+offset[2],3+offset[3],4+offset[4]},
  {0.5+offset[5],1.5+offset[6],2.5+offset[7],3.5+offset[8],4.5+offset[9]}
};

int load_index = 0;
double pos_seed = 36.0;
int mode_load = 0;
bool back_to_zero = false;
void loadServos_function();
void mode_load_system(int mode_load);

void loadServos_function() {
  myservo[0].attach(servoPin, 570, 2450);
  mode_load_system(mode_load);
  myservo[0].detach();
  delay(500);  // กันสั่นเล็กน้อย
}

void mode_load_system(int mode_load) {
  if(load_index == 0) myservo[0].attach(servoPin, 535, 2400);
  else myservo[0].attach(servoPin, 528, 2480);
  if(load_index == 1 && mode_load == 0) myservo[0].attach(servoPin,570, 2380);
  if(load_index == 2 && mode_load == 0) myservo[0].attach(servoPin,570, 2430);
  if(load_index == 3) myservo[0].attach(servoPin, 580, 2480);
  if(load_index == 4) myservo[0].attach(servoPin, 620, 2480);
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
        myservo[0].attach(servoPin, 535, 2400);
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



void setup() {
  Serial.begin(115200);   
   myservo[2].attach(22, 400, 2400);
  // myservo[0].attach(servoPin, 528, 2480);
  // myservo[0].attach(servoPin, 490, 2400);
  // myservo[0].attach(servoPin, 545, 2450);
}
//กว้างสุด 9
//แคบสุด 48
void loop() {
  // myservo[0].write(0);
  // delay(5000);
  // myservo[0].write(180);
  // if (Serial.available() > 0) {
  //   char c = Serial.read();
  //   if (c == '1') {
  //     // myservo[0].write(0);
  //     if(back_to_zero == false) loadServos_function();
  //   }
  //   if (c == '0') {
  //     // myservo[0].write(180);
  //     if(back_to_zero)loadServos_function();
  //     back_to_zero = false;
  //   }
  // }
  myservo[2].write(0);
  delay(5000);
  myservo[2].write(180);
}
