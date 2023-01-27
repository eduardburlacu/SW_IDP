#include "header.h"

 // Create objects for motor control (DC and Servo)

Adafruit_MotorShield  AFMS   = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft  = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);
<<<<<<< HEAD
NewPing sonarFront(pinFrontTrig, pinFrontEcho,RANGE_FRONT);
NewPing sonarSide(pinSideTrig, pinSideEcho, RANGE_SIDE);
Servo servoArm;
Servo servoClaw;
=======
Servo servo;
>>>>>>> parent of 32c0072 (corrected bugs)

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
<<<<<<< HEAD
=======

void junction_detector(void)
{
  if(is_junction==false)
  {
    /*
    Not necessary anymore.
    if (JUNCTION[0] == 0 && JUNCTION[1] == 0 ) { is_junction = false; }
    */
    
>>>>>>> parent of 32c0072 (corrected bugs)

void turnLeft(){
  motorLeft -> setSpeed(ROT_SPEED);
  motorRight-> setSpeed(ROT_SPEED);
  motorLeft -> run(BACKWARD);
  motorRight-> run(FORWARD);
  unsigned long start_time=millis();
  while(true){
    if(millis()-start_time>=TAU){
      break;
    }
    motorLeft  -> run(FORWARD);
    motorLeft  -> setSpeed(TOP_SPEED);
    motorRight -> setSpeed(TOP_SPEED);
  }
}

void turnRight(){

  motorLeft -> setSpeed(ROT_SPEED);
  motorRight-> setSpeed(ROT_SPEED);
  motorLeft -> run(FORWARD);
  motorRight-> run(BACKWARD);
  unsigned long start_time=millis();
  while(true){
    if(millis()-start_time>=TAU){
      break;
    }
    motorLeft  -> run(FORWARD);
    motorLeft  -> setSpeed(TOP_SPEED);
    motorRight -> setSpeed(TOP_SPEED);
  }
}

void junction_detector(void)
{
  switch(state){
    case 0:
      break;
    
    case 1:
    if(JUNCTION[0]==1 && JUNCTION[1]==1)
    {
      motorLeft -> setSpeed(0);
      motorRight-> setSpeed(0);
      turnRight();
      state=2;
    } else{
      Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case1. JUNCTION= ");
      Serial.print(JUNCTION[0]);
      Serial.print(JUNCTION[1]);
      }
      break;
    
    case 2:
      if (JUNCTION[0]==0 && JUNCTION[1]==1)
     {
        if(cube_to_destination)
        {
        motorLeft -> setSpeed(0);
        motorRight-> setSpeed(0);
        turnRight();
        }
      }else{
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case2. JUNCTION= ");
        Serial.print(JUNCTION[0]);
        Serial.print(JUNCTION[1]);     
      }
      break;
    
    case 3:
      if (JUNCTION[0]==1 && JUNCTION[1]==0)
      {
        motorLeft -> setSpeed(0);
        motorRight ->setSpeed(0);
        ultrasonic_lookup("side");
        if (cubeDetectedSide){
          turnLeft();
        } else{
          motorLeft -> setSpeed(TOP_SPEED);
          motorRight-> setSpeed(TOP_SPEED);
          }
      }else{
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case4. JUNCTION= ");
        Serial.print(JUNCTION[0]);
        Serial.print(JUNCTION[1]);
      }
      break;

    case 4:
      break;
    
    case 5:
    if(JUNCTION[0]==1 && JUNCTION[1]==0)
    {
      motorLeft -> setSpeed(0);
      motorRight-> setSpeed(0);
      ultrasonic_lookup("side");
      if (cubeDetectedSide){
        turnRight();
      } else{
          motorLeft -> setSpeed(TOP_SPEED);
          motorRight-> setSpeed(TOP_SPEED);
          }
    }
    else{
      Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case5. JUNCTION= ");
      Serial.print(JUNCTION[0]);
      Serial.print(JUNCTION[1]);
    }
      break;
    
    case 6:
      if(JUNCTION[0]==0 && JUNCTION[1]==1)
      {
        motorLeft -> setSpeed(0);
        motorRight-> setSpeed(0);
        if (cube_to_destination){
          turnRight();
        } else{
          motorLeft -> setSpeed(TOP_SPEED);
          motorRight-> setSpeed(TOP_SPEED);
          }
      }
      else{
      Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case6. JUNCTION= ");
      Serial.print(JUNCTION[0]);
      Serial.print(JUNCTION[1]);
      } 
      break;
  }
}

<<<<<<< HEAD
void ultrasonic_lookup(char* sensor)
{ 
    /*
    Performs the search for cubes in the robot is in the important region. First, it searches with the frontal sensor. If it is a cube frontways, 
    it will stop at the corresponding (distance to the cube) - (pickup_distance) for the collection mechanism.
    */

    if(sensor=="front")
      {
      unsigned long time = sonarFront.ping_median(NUM_READ);
      if(time != 0){
        unsigned int  dist = sonarFront.convert_cm(time);
        cubeDetectedFront=true;
        cubeDistance=dist;
      }
    else if(sensor=="side")
          {
      unsigned long time = sonarSide.ping_median(NUM_READ);
      if (time != 0){
        unsigned int dist = sonarSide.convert_cm(time);
        cubeDetectedSide=true;
        cubeDistance=dist;}
          }
      }  
    else{Serial.println("Error in ultrasonic lookup, invalid key. Valid keys are: front , side ."); }
}

void leave_box()
{
=======
void leave_box(uint8_t step_size=10){
>>>>>>> parent of 32c0072 (corrected bugs)
  if (is_in_box == false){ Serial.println("Error in leave_box, the state of the robot is_in_box is not true as expected"); }
  uint8_t current=0;
<<<<<<< HEAD
  motorLeft  -> setSpeed(TOP_SPEED);
  motorRight -> setSpeed(TOP_SPEED);
  motorLeft  -> run(FORWARD);
  motorRight -> run(FORWARD);
  do{
    junction_detector();  
  } while( JUNCTION[0]=='0' && JUNCTION[1]=='0' );
  /*
  motorLeft  -> setSpeed(0);
  motorRight -> setSpeed(0);
  junction_detector();
  */
  is_in_box=false; 
  }
}
/*
// to be called when robot is set distance from block
void cube_retrieval() {
  //slow down before reaching block
  unsigned int x = final_distance_reading();
  while (x > pickup_distance) {
    if (speedLeft != 127/2) {
      TOP_SPEED = TOP_SPEED/2;
      speedLeft = TOP_SPEED;
      speedRight = TOP_SPEED;
    }
    line_follower();
    x=final_distance_reading();
  }
  //stop robot
  motorLeft  -> setSpeed(0);
  motorRight -> setSpeed(0);
  speedLeft = 0;
  speedRight = 0;
  //descend claw
  servoArm.write(arm_down);
  //close claw 
  servoClaw.write(claw_close);
  //lift claw
  servoArm.write(arm_up);
=======
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
>>>>>>> parent of 32c0072 (corrected bugs)
}
*/
void line_follower(bool is_region=false){
  /*
  In an iteration, this pipeline function does the following: 
    1. setting the state of the IR sensors
    2. checks if there is a junction.                           !!!!!!!!!! TO ADD: INTEGRATE ULTRASONIC SENSORS INTEGRATION !!!!!!!!!!!!
    3. computes the required action based on the current output relative to the desired state ( e=0 state ).
  */
  get_state();
  junction_detector();
  if (is_region){
    ultrasonic_lookup("front");
  }
  get_error();
  float x = get_control_signal();
<<<<<<< HEAD
  if (speedLeft!=TOP_SPEED + DELTA - x && speedRight!=TOP_SPEED - DELTA - x)
    {
    motorLeft  -> setSpeed(TOP_SPEED + DELTA - x);
    motorRight -> setSpeed(TOP_SPEED - DELTA - x);  
    }
=======
  if (speedLeft!=255/2 + DELTA - x){
    motorLeft  -> setSpeed(TOP_SPEED + DELTA - x);
    motorRight -> setSpeed(TOP_SPEED- DELTA - x);  }
>>>>>>> parent of 32c0072 (corrected bugs)
}

void setup() {
  Serial.begin(9600);
  AFMS.begin();
<<<<<<< HEAD
  //Sensors setup step
    //IR sensors for line following
=======
>>>>>>> parent of 32c0072 (corrected bugs)
  pinMode(pinL, INPUT);
  pinMode(pinR, INPUT);
  pinMode(pinLL,INPUT);
  pinMode(pinRR,INPUT);
<<<<<<< HEAD
  servoArm.attach(pinArm);
  servoClaw.attach(pinClaw);
      // Ultrasonic sensors for object detection.INPUT
  pinMode(pinFrontTrig, OUTPUT);
  pinMode(pinFrontEcho, INPUT);
  pinMode(pinSideTrig,  OUTPUT);
  pinMode(pinSideEcho,  INPUT);
      //Motors setup step
  motorLeft -> setSpeed(0);
  motorRight -> setSpeed(0);
  motorLeft -> run(FORWARD);
  motorRight -> run(FORWARD);
}

void loop() 
{
//Leave the box at start and after the cube has been placed correctly. !!!!!!!!!!!!NB turn 180 after the cube has been placed successfully
  if(is_in_box && (!cube_to_destination) )  
  { 
  leave_box(); 
  }
  do{ 
    line_follower();
    delay(REFRESH_TIME);
    }while( !is_region );
  
  while(!cubeDetectedFront ) 
  {  
    line_follower(true); 
    }






=======
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(REFRESH_TIME);
>>>>>>> parent of 32c0072 (corrected bugs)
}
