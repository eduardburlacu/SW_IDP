#include "header.h"

 // Create objects for motor control (DC and Servo)

Adafruit_MotorShield  AFMS   = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft  = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);
Servo servo;
Servo servoArm;
Servo servoClaw;

void accelerate(uint8_t initial_speed=0, uint8_t final_speed= TOP_SPEED, uint8_t step_size=5, bool reverse=false)
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
  if(is_junction==false)
  {
    /*
    Not necessary anymore.
    if (JUNCTION[0] == 0 && JUNCTION[1] == 0 ) { is_junction = false; }
    */
    

    if (JUNCTION[0] == 1 && JUNCTION[1] == 0 ) 
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
      if(is_region){is_junction = true;} //!!!!!!!!Check this out for the 2nd intersection!!!!
      else if(is_in_box=true){
        motorLeft  -> setSpeed(0);
        motorRight -> setSpeed(0);
        motorLeft  -> run(RELEASE);
        motorRight -> run(RELEASE);
        if (to_tunnel) { turnLeft(); } else{ turnRight(); }    
        is_junction = false;
        }
    }
    else {
      Serial.print("Error, JUNCTION HAS REACHED THE VALUES:");
      Serial.print(JUNCTION[0]);
      Serial.println(JUNCTION[1]);
    }
  }
}

void leave_box(uint8_t step_size=10){
  if (is_in_box == false){ Serial.println("Error in leave_box, the state of the robot is_in_box is not true as expected"); }
  uint8_t current=0;
  motorLeft -> run(FORWARD);
  motorLeft -> run(FORWARD);
  do{
    if (current<TOP_SPEED/4)
    {current += step_size;
    motorLeft  -> setSpeed(current);
    motorRight -> setSpeed(current);  }
  } while( JUNCTION[0]=='0' && JUNCTION[1]=='0' );
  motorLeft -> setSpeed(0);
  motorRight -> setSpeed(0);
  motorLeft  -> run(RELEASE);
  motorRight -> run(RELEASE);
  junction_detector();
  is_in_box=false;
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
  /*
  In an iteration, this pipeline function does the following: 
    1. setting the state of the IR sensors
    2. checks if there is a junction.                           !!!!!!!!!! TO ADD: INTEGRATE ULTRASONIC SENSORS INTEGRATION !!!!!!!!!!!!
    3. computes the required action based on the current output relative to the desired state ( e=0 state ).
  */
  get_state();
  junction_detector();
  get_error();
  float x = get_control_signal();
  if (speedLeft!=255/2 + DELTA - x){
    motorLeft  -> setSpeed(TOP_SPEED + DELTA - x);
    motorRight -> setSpeed(TOP_SPEED- DELTA - x);  }
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
  delay(REFRESH_TIME);
}
