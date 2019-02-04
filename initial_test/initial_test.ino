#include <Adafruit_INA219.h> 
#include <Timer.h>

#define DRIVE0_PIN 6
#define DRIVE1_PIN 2
#define DRIVE2_PIN 5
#define DRIVE3_PIN 4
#define PISTON_POS       A0
#define P_PID_LIMIT      (300)
#define DB               (2)
#define ARRAY_SIZE       (1)
#define LOOP_DELAY       (0.1)
#define ERR_MAX          100
#define P_GAIN           (5)
#define I_GAIN           0.01 //(0.0004*LOOP_DELAY) //0.00005*LOOP_DELAY;; //0.0001*LOOP_DELAY;
#define D_GAIN           0//(0.2)
#define ALPHA            (0.004)
#define VALVE_ON         (160)
#define VALVE_OFF        (100)
#define P_GAIN_CURRENT0  (0.5)
#define I_GAIN_CURRENT0  (0.001)

int zero_pos = 0;
Adafruit_INA219 sensor219; 
Timer control_timer;
float i_pid_current = 0;
float p_pid_current = 0;
float err_current = 0;
float desired_current = 200;
float cmd_current = 0;
float current = 0;
float filter_pos[3] = {0};
float prev_pos[2] = {0};
float b[3] = {0.09763107, 0.19526215, 0.09763107}; //{0.64135154, 1.28270308, 0.64135154};
float a[3] = {1,  -0.94280904,  0.33333333};
float current_array[10] = {0};
  
void setup() {
  sensor219.begin();
  Serial.begin(2000000);
  pinMode(DRIVE0_PIN, OUTPUT);
  pinMode(DRIVE1_PIN, OUTPUT);
  pinMode(DRIVE2_PIN, OUTPUT);
  pinMode(DRIVE3_PIN, OUTPUT);
  digitalWrite(DRIVE1_PIN, LOW);
  digitalWrite(DRIVE3_PIN, LOW);
  int prescalerVal = 0x07;
  TCCR0B &= ~prescalerVal;
  prescalerVal = 3;
  TCCR0B |= prescalerVal;
  analogWrite(DRIVE2_PIN, VALVE_ON);
  delay(5000);
  zero_pos = analogRead(PISTON_POS); 
  analogWrite(DRIVE2_PIN, VALVE_OFF);
  control_timer.every(1, control_valve, -1);
  control_timer.every(1000, change_target, -1);
}

void change_target() {
  if (desired_current == 200) {
    desired_current = 400;
  }
  else {
    desired_current = 200;
  }
}

void control_valve() {
  float adc_current = sensor219.getCurrent_mA();
  double current_sum = 0;
  for (int i = 0; i < 10 - 1; i++) {
    current_array[i] = current_array[i+1];
    current_sum += current_array[i];
  }
  current_array[9] = adc_current;
  current_sum += adc_current;
  current = current_sum/10;
  filter_pos[0] = -a[1]*filter_pos[1] -a[2]*filter_pos[2]; 
  filter_pos[0] += b[0]*current + b[1]*prev_pos[0] + b[2]*prev_pos[1]; //current_pos * alpha + (1 - alpha) * prev_pos;
  prev_pos[1] = prev_pos[0];
  prev_pos[0] = current;
  filter_pos[2] = filter_pos[1];
  filter_pos[1] = filter_pos[0];

  err_current = desired_current - filter_pos[0]; //current;
  i_pid_current += I_GAIN_CURRENT0 * err_current;
  p_pid_current = P_GAIN_CURRENT0 * err_current;
  cmd_current = i_pid_current + p_pid_current;
  if (cmd_current < 0) {
    cmd_current = 0;
  }
  analogWrite(DRIVE0_PIN, (int)cmd_current);

  Serial.print(desired_current);
  Serial.print(", ");
  Serial.print(filter_pos[0]);
  Serial.print(", ");
  Serial.print(cmd_current);  
  Serial.print("\n");

//  static int fadeValue = 0;
//  float current = sensor219.getCurrent_mA();
//  analogWrite(DRIVE0_PIN, fadeValue);
//  analogWrite(DRIVE2_PIN, fadeValue);
//  fadeValue += 5;
//  if (fadeValue > 500) {
//    fadeValue = 0;
//  }
//  Serial.println(current);
}

void loop() {
  float current_pos = 0;
  int i = 0;
  float desired_pos = 30;
  float err_pos = 0;
  float prev_err_pos = 0;
  float prev_pos[2] = {0};
  long cmd_pos = 0;
  float i_pid_current = 0;
  float p_pid_current = 0;
  float i_pid = 0;
  float p_pid = 0;
  float d_pid = 0;
  float data0[ARRAY_SIZE];
  float data1[ARRAY_SIZE];
  float filter_pos[3] = {0};
  float b[3] = {0.09763107, 0.19526215, 0.09763107}; //{0.64135154, 1.28270308, 0.64135154};
  float a[3] = {1,  -0.94280904,  0.33333333};
  float inc = 0.2;
  bool pos_reached = false;
  float alpha = 0.7;
  int j = 0;
  bool first_flag = false;
  float ke_gain = 1;
  float err_clamp = 0;
  float dither = 0.25;
  int fadeValue = 0;
  float current = 0;
  float desired_current = 300;
  float err_current = 0;
  float cmd_current = 0;

  control_timer.update();
//  while (true) {
//    //current_pos = (float)(analogRead(PISTON_POS) - zero_pos);     // read the input pin
//    current_pos = sensor219.getCurrent_mA();
//    current = current_pos;
//    filter_pos[0] = -a[1]*filter_pos[1] -a[2]*filter_pos[2]; 
//    filter_pos[0] += b[0]*current_pos + b[1]*prev_pos[0] + b[2]*prev_pos[1]; //current_pos * alpha + (1 - alpha) * prev_pos;
//    prev_pos[1] = prev_pos[0];
//    prev_pos[0] = current_pos;
//    filter_pos[2] = filter_pos[1];
//    filter_pos[1] = filter_pos[0];
////    Serial.print(current_pos);             // debug value
////    Serial.print("\n");
//    prev_err_pos = err_pos;
//    err_pos = desired_pos - filter_pos[0];
//    err_current = desired_current - filter_pos[0];
////    if (abs(err_pos) > ERR_MAX) {
////      if (err_pos > 0) {
////        err_clamp = ERR_MAX;
////      }
////      else {
////        err_clamp = -ERR_MAX;
////      }
////    }
////    else {
////      err_clamp = err_pos;
////    }
////    ke_gain = (exp(ALPHA * err_clamp) + exp(-ALPHA * err_clamp))/2;
////    if (ke_gain > 3) {
////      ke_gain = 3;
////    }
////    err_pos *= ke_gain;
//
//    if (j++ == 1000)
//    {
//      if (desired_pos == 30) {
//        desired_pos = 50;
//        desired_current = 200;
//      }
//      else if (desired_pos == 50) {
//        desired_pos = 30;
//        desired_current = 600;
//      }
//      j = 0;
//    }
//      
//      /*if (pos_reached) {
//        if (j++ == 2000)
//        {
//          pos_reached = false;
//          j = 0;
//        }
//      }
//      if (pos_reached == false) {
//        desired_pos += inc;      
//      }
//      if (desired_pos > 450) {
//        inc = -0.2;
//        pos_reached = true;
//        first_flag = true;
//      }
//      if (desired_pos < 300 & first_flag) {
//        inc = 0.2;
//        pos_reached = true;
//      }*/            
//
//    
////    cmd_pos = 0;
////    stepper_left.moveTo((long)cmd_pos);
////    stepper_left.runToPosition();
////    cmd_pos = -200;
////    stepper_left.moveTo((long)cmd_pos);
////    stepper_left.runToPosition();
//    
//
//    i_pid += I_GAIN * err_pos;
//    p_pid = P_GAIN * err_pos;
//    d_pid = D_GAIN * (err_pos - prev_err_pos);
//    if (i_pid > P_PID_LIMIT) {
//      i_pid = P_PID_LIMIT;
//    }
//    if (i_pid < -P_PID_LIMIT) {
//      i_pid = -P_PID_LIMIT;
//    }
//    cmd_pos = p_pid + i_pid + d_pid;
//
//    i_pid_current += I_GAIN * err_current;
//    p_pid_current = P_GAIN * err_current;
//    cmd_current = i_pid_current + p_pid_current;
//
////    if (abs(err_pos) < 20) {
////      cmd_pos += 0.01;
////    }
////    else {
////      if (cmd_pos > 0) {
////        cmd_pos += 20;
////      }
////      else {
////        cmd_pos -= 20;
////      }
////    }
////
////    cmd_pos += dither;
////    dither = -dither;
//    
////    if (cmd_pos > 255) {
////      cmd_pos = 255;
////    }
//    if (cmd_current < 0) {
//      cmd_current = 0;
//    }
////    analogWrite(DRIVE0_PIN, (int)cmd_current);
////    delay(1);
//
//    
//    data0[i] = desired_current;
//    data1[i] = filter_pos[0]; // 
//
////    Serial.print(data0[i]);
////    Serial.print(", ");
////    Serial.print(data1[i]);
////    Serial.print("\n");
//          
////    if (i++ == ARRAY_SIZE) {
////      i = 0;
////      if (desired_pos%20 == 0) {
////        for(int j = 0; j < ARRAY_SIZE; j++)
////        {
////          Serial.print(data0[j]);
////          Serial.print(", ");
////          Serial.print(data1[j]);
////          Serial.print("\n");
////        }        
////      }
////    }
////
////    analogWrite(DRIVE0_PIN, fadeValue);
////    analogWrite(DRIVE2_PIN, fadeValue);
////    fadeValue += 5;
////    if (fadeValue > 500) {
////      fadeValue = 0;
////    }
////    Serial.println(current);
////    delay(500);
//  }
}
