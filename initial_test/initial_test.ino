#include <Event.h>
#include <Timer.h>

#include <AccelStepper.h>
#include "Timer.h"

// used pins
#define MOTOR_EN         8  //D11
#define MOTOR_L_PIN_DIR  4   //D6
#define MOTOR_L_PIN_PUL  3   //D5
#define LED              13
#define THROTTLE_SPEED   (80000)
#define THROTTLE_ACC     (8000000)
#define PISTON_POS       A0
#define P_PID_LIMIT      (200)
#define DB               (0)

AccelStepper stepper_left(AccelStepper::DRIVER, MOTOR_L_PIN_PUL, MOTOR_L_PIN_DIR);
Timer monitor_timer;

void rezero() {
  digitalWrite(MOTOR_EN, HIGH);
  stepper_left.setCurrentPosition(0);
}

void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, HIGH);  //first thing: disable motor so no drive
  delay(2000);
  stepper_left.setMaxSpeed(THROTTLE_SPEED);
  stepper_left.setAcceleration(THROTTLE_ACC);
  rezero();
}

void loop() {
  int current_pos = 0;
  int i = 0;
  float desired_pos = 0;
  float err_pos = 0;
  long cmd_pos = 0;
  float p_gain = 0.1;
  float i_gain = 0.0001;
  float i_pid = 0;
  float p_pid = 0;

  stepper_left.moveTo(0);
  stepper_left.runToPosition();
  delay(5000);
    
  while (true) {
    desired_pos += 0.1;
    if (desired_pos > 800) {
      desired_pos = 0;
    }
    
//    cmd_pos = 0;
//    stepper_left.moveTo((long)cmd_pos);
//    stepper_left.runToPosition();
//    cmd_pos = -200;
//    stepper_left.moveTo((long)cmd_pos);
//    stepper_left.runToPosition();
    
    current_pos = analogRead(PISTON_POS);     // read the input pin
    //Serial.println(current_pos);             // debug value
    err_pos = -(float)(desired_pos - (float)current_pos);
    i_pid += i_gain * err_pos;
    p_pid = p_gain * err_pos;
    if (p_pid > P_PID_LIMIT) {
      p_pid = P_PID_LIMIT;
    }
    if (p_pid < -P_PID_LIMIT) {
      p_pid = -P_PID_LIMIT;
    }
    cmd_pos = p_pid + i_pid;
    stepper_left.moveTo((long)cmd_pos);
    stepper_left.runToPosition();
    delay(1);
//    Serial.println(i_pid, "i_pid");
//    Serial.println(p_pid);
  }
}
