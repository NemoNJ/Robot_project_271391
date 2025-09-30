#include <Arduino.h>
#define MOTOR_1  16
#define MOTOR_2  17

void setup()
{
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
}

void loop(){
  digitalWrite(MOTOR_1, LOW);
  digitalWrite(MOTOR_2,HIGH);
  delay(5000); 
  digitalWrite(MOTOR_1, LOW);
  digitalWrite(MOTOR_2,LOW);
}