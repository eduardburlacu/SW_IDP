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
    motorRight-> run(FORWARD);  }
    else {
    motorLeft -> run(BACKWARD);
    motorRight-> run(BACKWARD); }
  uint8_t current=initial_speed;
  while(current!=final_speed) {
    motorLeft  -> setSpeed(current);
    motorRight -> setSpeed(current); 
    current += step_size;
    delay(10);  }
}

void junction_detector(void)
{
    if (JUNCTION[0] == 0 && JUNCTION[1] == 0 ) { is_junction = false; }

    else if (JUNCTION[0] == 1 && JUNCTION[1] == 0 ) 
    { 
      motorLeft  -> setSpeed(0);
      motorRight -> setSpeed(0);
      motorLeft  -> run(RELEASE);
      motorRight -> run(RELEASE);
      turnLeft();
      is_junction = true;
    }

    else if  (JUNCTION[0] == 0 && JUNCTION[1] == 1 ){
      motorLeft  -> setSpeed(0);
      motorRight -> setSpeed(0);
      motorLeft  -> run(RELEASE);
      motorRight -> run(RELEASE);
      turnRight();
      is_junction = true;
    }

    else if  (JUNCTION[0] == 1 && JUNCTION[1] == 1 )
    {
      if(is_region){is_junction = false;}
      else{
        motorLeft  -> setSpeed(0);
        motorRight -> setSpeed(0);
        motorLeft  -> run(RELEASE);
        motorRight -> run(RELEASE);
        if (to_tunnel) { turnLeft(); } else{turnRight(); }    // !!!!!!!!!!!!!!!!!!!!! Check this out tomorrow on map !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        is_junction = false;
        }
    }
    else {
      Serial.print("Error, JUNCTION HAS REACHED THE VALUES:");
      Serial.print(JUNCTION[0]);
      Serial.println(JUNCTION[1]);
    }
}

void setup() {
  Serial.begin(9600);
  AFMS.begin();
  pinMode(pinL, INPUT);
  pinMode(pinR, INPUT);
  pinMode(pinLL,INPUT);
  pinMode(pinRR,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2);
}
