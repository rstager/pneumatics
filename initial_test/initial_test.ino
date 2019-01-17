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
#define DB               (10)
#define ARRAY_SIZE       (1)
#define LOOP_DELAY       (1)

AccelStepper stepper_left(AccelStepper::DRIVER, MOTOR_L_PIN_PUL, MOTOR_L_PIN_DIR);
Timer monitor_timer;
int zero_pos = 0;

void rezero() {
  digitalWrite(MOTOR_EN, HIGH);
  stepper_left.setCurrentPosition(0);
}

void setup() {
  Serial.begin(2000000);
  pinMode(LED, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, HIGH);  //first thing: disable motor so no drive
  delay(2000);
  stepper_left.setMaxSpeed(THROTTLE_SPEED);
  stepper_left.setAcceleration(THROTTLE_ACC);
  rezero();
  zero_pos = analogRead(PISTON_POS);     // read the input pin
}

void loop() {
  int current_pos = 0;
  int i = 0;
  float desired_pos = 10;
  float err_pos = 0;
  float prev_pos = 0;
  long cmd_pos = 0;
  float p_gain = 0.05*LOOP_DELAY;
  float i_gain = 0.0001*LOOP_DELAY;
  float i_pid = 0;
  float p_pid = 0;
  float data0[ARRAY_SIZE];
  float data1[ARRAY_SIZE];
  float filtered_pos = 0;
  stepper_left.moveTo(0);
  stepper_left.runToPosition();
  delay(5000);

  float inc = 0.2;
  bool pos_reached = false;
  float alpha = 0.9;
  int j = 0;
  bool first_flag = false;

  while (true) {
    current_pos = analogRead(PISTON_POS) - zero_pos;     // read the input pin
    //filtered_pos = current_pos * alpha + (1 - alpha) * prev_pos;
    //prev_pos = current_pos;
    //Serial.println(current_pos);             // debug value
    err_pos = -(desired_pos - (float)current_pos) ;
    data0[i] = (float)current_pos;
//    if (desired_pos == 50 && abs(err_pos) < 5) {
//      pos_reached = true;
//    }
      
      if (pos_reached) {
        if (j++ == 2000)
        {
          pos_reached = false;
          j = 0;
        }
      }

      if (pos_reached == false) {
        desired_pos += inc;      
      }
      
      if (desired_pos > 600) {
        inc = -0.2;
        pos_reached = true;
        first_flag = true;
      }
      if (desired_pos < 200 & first_flag) {
        inc = 0.2;
        pos_reached = true;
      }            

    
//    cmd_pos = 0;
//    stepper_left.moveTo((long)cmd_pos);
//    stepper_left.runToPosition();
//    cmd_pos = -200;
//    stepper_left.moveTo((long)cmd_pos);
//    stepper_left.runToPosition();
    

    i_pid += i_gain * err_pos;
    p_pid = p_gain * err_pos;
    if (p_pid > P_PID_LIMIT) {
      p_pid = P_PID_LIMIT;
    }
    if (p_pid < -P_PID_LIMIT) {
      p_pid = -P_PID_LIMIT;
    }
    cmd_pos = p_pid + i_pid;
//    float flag = err_pos * prev_err_pos;
//    if (flag < 0 && abs(err_pos - prev_err_pos) > 10) {
//      if (err_pos > 0) {
//        cmd_pos += DB;
//      }
//      else {
//        cmd_pos -= DB;
//      }
//    }
    data1[i] = cmd_pos;

    stepper_left.moveTo((long)cmd_pos);
    stepper_left.runToPosition();
    delay(LOOP_DELAY);

    Serial.print(data0[i]);
    //Serial.print(", ");
    //Serial.print(-data1[i]*10);
    Serial.print("\n");
          
//    if (i++ == ARRAY_SIZE) {
//      i = 0;
//      if (desired_pos%20 == 0) {
//        for(int j = 0; j < ARRAY_SIZE; j++)
//        {
//          Serial.print(data0[j]);
//          Serial.print(", ");
//          Serial.print(data1[j]);
//          Serial.print("\n");
//        }        
//      }
//    }
  }
}
