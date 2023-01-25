#include "header.h"

 // Create objects for motor control (DC and Servo)

Adafruit_MotorShield  AFMS   = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft  = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);
Servo servo;
Servo servoArm;
Servo servoClaw;

void accelerate(uint8_t initial_speed=0, uint8_t final_speed=255/2, uint8_t step_size=5, bool reverse=false)
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
        if (to_tunnel) { turnLeft(); } else{turnRight(); }    // !!!!!!!!!!!!!!!!!!!!! Check this out tomorrow on map, it may be the opposite !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        is_junction = false;
        }
    }
    else {
      Serial.print("Error, JUNCTION HAS REACHED THE VALUES:");
      Serial.print(JUNCTION[0]);
      Serial.println(JUNCTION[1]);
    }
}

// to be called when robot is set distance from block
void cube_retrieval() {
  //slow down before reaching block
  while (final_distance_reading() > pickup_distance) {
    if (speedLeft != 127/2)) {
      motorLeft  -> setSpeed(127/2);
      motorRight  -> setSpeed(127/2);
      speedLeft = 127/2;
      speedRight = 127/2;
    }
  }
  //stop robot
  motorLeft  -> setSpeed(0);
  motorRight  -> setSpeed(0);
  motorLeft  -> run(RELEASE);
  motorRight -> run(RELEASE);
  speedLeft = 0;
  speedRight = 0;
  //descend claw
  servoArm.write(arm_down);
  //close claw 
  servoClaw.write(claw_close);
  //lift claw
  servoArm.write(arm_up);
}

void line_follower(void){
  get_state();
  junction_detector();
  get_error();
  float x = get_control_signal();
  motorLeft -> setSpeed(255/2 + DELTA - x);
  motorRight -> setSpeed(255/2- DELTA - x);  
  delay(REFRESH_TIME);
}

void setup() {
  Serial.begin(9600);
  AFMS.begin();
  pinMode(pinL, INPUT);
  pinMode(pinR, INPUT);
  pinMode(pinLL,INPUT);
  pinMode(pinRR,INPUT);
  servoArm.attach(pinArm);
  servoClaw.attach(pinClaw);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2);
}
