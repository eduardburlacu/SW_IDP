#include <Adafruit_MotorShield.h>
#include "Wire.h"
#include <Servo.h>

// Define the relevant state variables
extern bool  is_junction = false;  // True if the algorithm has found a junction or + intersection.
extern bool  is_region = false;    // True if the robot has reached the region where cubes are expected.        !!!!!!! ADD STH SO THAT IT GETS TRIGGERED !!!!!!!!!!!
extern bool  is_tunnel=false;      // When inside the tunnel, becomes on by triggering the photodiode receptor. !!!!!!! ADD STH SO THAT IT GETS TRIGGERED !!!!!!!!!!!
extern bool  to_tunnel=false;      // When the return path is by the tunnel.                                    !!!!!!! ADD STH SO THAT IT GETS TRIGGERED !!!!!!!!!!!
extern int   REFRESH_TIME=10; 		 // Time in miliseconds after which a new update is provided
extern int   NUM_READ = 8;				 // Number of points to use for average
extern int   LINE[]     = {0,0};	 // Variable encoding the state of the 2 IR sensors for line following.
extern int   JUNCTION[] = {0,0};   // Variable encoding the state of the 2 IR sensors for junction detection.
extern int   speedLeft = 0;        // Variable for the speed of the left  DC Motor, range 0->255
extern int   speedRight =0;        // Variable for the speed of the right DC Motor, range 0->255
extern float THETA = 0.0;			     // Angle of rotation relative to the line to be followed.
extern float PREV_THETA = 0.0;     // Previous step -> angle of rotation relative to the line to be followed.
extern float TOTAL_THETA = 0.0;		 // Discrete integral of rotation relative to the line to be followed.
extern float Kd, Kp, Ki;           // PID controller parameters. Need to be found analitically after the mechanics is done!!
extern float TAU;                  // The required time to perform a 90 deg turn.
extern uint8_t DELTA;              // Compensation caused by the imbalance in weight of distribution in the robot. Make it positive if it drifts to the left!!
// Needs assigned value and tweaked tomorrow!!!!!!


// Define the pin numbers for sensors and motors. To be filled in with Andrew after they finish the electrical circuits!!

#define pinL  1
#define pinR  2
#define pinLL 3
#define pinRR 4
#define pinUltrasonicFront
#define pinUltrasonicSide
#define pinRedPD
#define pinBluePD
#define pinRedLed
#define pinBlueLed
#define pinMotorLeft
#define pinMotorRight
#define pinPotServo           // Analog pin used to connect the potentiometer


// Prototypes for functions

float get_control_signal(void);
float moving_avg(int N, int len,float v[]);
float moving_avg(int len, float v[]);

uint8_t measure(int pin);

void accelerate(uint8_t initial_speed, uint8_t final_speed, uint8_t step_size, bool reverse);
void blinkLed(void);
void get_error(void);
void get_state(void);
void junction_search_cube(void);
void junction_detector(void);
void lift_cube(void);
void line_follower(void);
void tunel_navigation(void);
void search_cube(void);
void turnLeft(void);
void turnLeft(int angle);
void turnRight(void);
void turnRight(int angle);

// Some general purpose functions

float moving_avg(int N, int len, float v[])
{
    /*
    Use it to get rid of random fluctuations in reading.
    N is the number of points considered to the average.
    len is the length of v
    v[] stores the last recorded values of a variable to be averaged.
    if N < len , only considers the last N readings in v[].
    */
    float average_total = 0.0;
    for (int index = len- N; index < len; index++) {
        average_total += v[index];
    }
    average_total /= N;
    return average_total;
}

float moving_avg(int len, float v[])
{
    float average_total = 0.0;
    for (int index = 0; index < len; index++) {
        average_total += v[index];
    }
    average_total /= len;
    return average_total;
}

void get_error(void)
{ 
  /*
  Convention: shift to left leads to positive error.
  */
  PREV_THETA = THETA;
  if (LINE=="01"){ THETA = 1.0; }
  else if( LINE=="10" ){THETA = -1.0; }
  else {THETA = 0.0; }
}

float get_control_signal(void){ 
  // Used for the feedback loop of the parameter. The new value of the motor speed is set accordingly.
  return Kp * THETA + Ki * TOTAL_THETA + Kd * (THETA - PREV_THETA); }

void get_state(void)
{
  int v[4]={0,0,0,0};
  for (int i=0;i<NUM_READ;i++)
  {
  LINE[0]     += digitalRead(pinL);
  LINE[1]     += digitalRead(pinR);
  JUNCTION[0] += digitalRead(pinLL);
  JUNCTION[0] += digitalRead(pinRR);
  delay(5); 
  }
  for (int j=0;j<2;j++)
  {  
    LINE[j]     = (LINE[j]    + NUM_READ - 1)/NUM_READ;
    JUNCTION[j] = (JUNCTION[j]+ NUM_READ - 1)/NUM_READ;
    } 
}
