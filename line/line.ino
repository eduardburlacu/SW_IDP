#include "header.h"
 // Create objects for motor control (DC and Servo)

Adafruit_MotorShield  AFMS   = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft  = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);
Servo servo;

void accelerate(uint8_t initial_speed=0, uint8_t final_speed=255, uint8_t step_size=5, bool reverse=false)
{
  motorLeft -> setSpeed(initial_speed);
  motorRight-> setSpeed(initial_speed);
  if (!reverse){
    motorLeft -> run(FORWARD);
    motorRight-> run(FORWARD);
    } else {
    motorLeft -> run(BACKWARD);
    motorRight-> run(BACKWARD);
    }
  uint8_t current=initial_speed;
  while(current!=final_speed)
  {
    motorLeft -> setSpeed(current);
    motorRight -> setSpeed(current); 
    current += step_size;
    delay(10);
  }
}



void setup() {
  AFMS.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2);
}
