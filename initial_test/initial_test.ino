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
#define DB               (2)
#define ARRAY_SIZE       (1)
#define LOOP_DELAY       (0.5)
#define ERR_MAX          2000

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
  float current_pos = 0;
  int i = 0;
  float desired_pos = 10;
  float err_pos = 0;
  float prev_err_pos = 0;
  float prev_pos[2] = {0};
  long cmd_pos = 0;
  float p_gain = 0.035*LOOP_DELAY*2;
  float i_gain = p_gain / 700; //0.00005*LOOP_DELAY;; //0.0001*LOOP_DELAY;
  float d_gain = 0.02*LOOP_DELAY*2;
  float i_pid = 0;
  float p_pid = 0;
  float d_pid = 0;
  float data0[ARRAY_SIZE];
  float data1[ARRAY_SIZE];
  float filter_pos[3] = {0};
  float b[3] = {0.02008337, 0.04016673, 0.02008337}; //{0.64135154, 1.28270308, 0.64135154};
  float a[3] = {1, -1.56101808,  0.64135154};

  stepper_left.moveTo(0);
  stepper_left.runToPosition();
  delay(5000);

  float inc = 0.2;
  bool pos_reached = false;
  float alpha = 0.7;
  int j = 0;
  bool first_flag = false;
  float ke_gain = 1;

  while (true) {
    current_pos = (float)(analogRead(PISTON_POS) - zero_pos);     // read the input pin
    filter_pos[0] = -a[1]*filter_pos[1] -a[2]*filter_pos[2]; 
    filter_pos[0] += b[0]*current_pos + b[1]*prev_pos[0] + b[2]*prev_pos[1]; //current_pos * alpha + (1 - alpha) * prev_pos;
    prev_pos[1] = prev_pos[0];
    prev_pos[0] = current_pos;
    filter_pos[2] = filter_pos[1];
    filter_pos[1] = filter_pos[0];
    //Serial.println(current_pos);             // debug value
    prev_err_pos = err_pos;
    err_pos = -(desired_pos - (float)filter_pos[0]) ;
    ke_gain = 1 + (exp(err_pos/ERR_MAX) + exp(-err_pos/ERR_MAX))/2;
    if (ke_gain > 3) {
      ke_gain = 3;
    }
    err_pos *= ke_gain;

    if (j++ == 5000)
    {
      if (desired_pos == 10) {
        desired_pos = 300;
      }
      else if (desired_pos == 300) {
        desired_pos = 450;
      }
      else if (desired_pos == 450) {
        desired_pos = 300;
      }
      j = 0;
    }
      
      /*if (pos_reached) {
        if (j++ == 2000)
        {
          pos_reached = false;
          j = 0;
        }
      }
      if (pos_reached == false) {
        desired_pos += inc;      
      }
      if (desired_pos > 450) {
        inc = -0.2;
        pos_reached = true;
        first_flag = true;
      }
      if (desired_pos < 300 & first_flag) {
        inc = 0.2;
        pos_reached = true;
      }*/            

    
//    cmd_pos = 0;
//    stepper_left.moveTo((long)cmd_pos);
//    stepper_left.runToPosition();
//    cmd_pos = -200;
//    stepper_left.moveTo((long)cmd_pos);
//    stepper_left.runToPosition();
    

    i_pid += i_gain * err_pos;
    p_pid = p_gain * err_pos;
    d_pid = d_gain * (err_pos - prev_err_pos);
    if (i_pid > P_PID_LIMIT) {
      i_pid = P_PID_LIMIT;
    }
    if (i_pid < -P_PID_LIMIT) {
      i_pid = -P_PID_LIMIT;
    }
    cmd_pos = p_pid + i_pid + d_pid;
//
//    if (abs(err_pos) < 5) {
//      cmd_pos = 0.01;
//    }
//    else {
//      if (cmd_pos > 0) {
//        cmd_pos += 0.1;
//      }
//      else {
//        cmd_pos -= 0.1;
//      }
//    }

    data0[i] = err_pos;
    data1[i] = cmd_pos; 

    stepper_left.moveTo((long)cmd_pos);
    stepper_left.runToPosition();
    delay(LOOP_DELAY);

    Serial.print(data0[i]);
    Serial.print(", ");
    Serial.print(data1[i]);
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
