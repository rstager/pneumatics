#include <Timer.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

#define DRIVE0_PIN 6
#define DRIVE1_PIN 2
#define DRIVE2_PIN 5
#define DRIVE3_PIN 4
#define PRESSURE0        A0
#define POS0             A3
#define DB               (819)
#define LOOP_DELAY       (0.1)
#define VALVE_ON         (160)
#define VALVE_OFF        (100)
#define P_PID_LIMIT      (3000)
#define P_GAIN           (50)
#define I_GAIN           0.2 //(0.0004*LOOP_DELAY) //0.00005*LOOP_DELAY;; //0.0001*LOOP_DELAY;
#define D_GAIN           0//(0.2)
  
int desired_pos = 125;
float current_pos = 0;
float err_pos = 0;
float prev_err_pos = 0;
float cmd_pos = 0;
int pressure0 = 0;
int pos0 = 0;
int zero_pos = 0;
uint32_t counter;
float i_pid = 0;
float p_pid = 0;
float d_pid = 0;
Timer control_timer;
Adafruit_MCP4725 dac;

void setup() {
  Serial.begin(2000000);
//  pinMode(DRIVE0_PIN, OUTPUT);
//  pinMode(DRIVE1_PIN, OUTPUT);
//  pinMode(DRIVE2_PIN, OUTPUT);
//  pinMode(DRIVE3_PIN, OUTPUT);
//  digitalWrite(DRIVE0_PIN, HIGH);
//  digitalWrite(DRIVE1_PIN, HIGH);
//  digitalWrite(DRIVE2_PIN, HIGH);
//  digitalWrite(DRIVE3_PIN, HIGH);
  dac.begin(0x60);
  delay(3000);
  zero_pos = analogRead(POS0); 
  control_timer.every(10, control_valve, -1);
  control_timer.every(9000, change_target, -1);
}

void change_target() {
  if (desired_pos == 125) {
    desired_pos = 150;
  }
  else {
    desired_pos = 125;
  }
}

void control_valve() {
  current_pos = -(float)(pos0 - zero_pos); 
  err_pos = desired_pos - current_pos;
  i_pid += I_GAIN * err_pos;
  p_pid = P_GAIN * err_pos;
  d_pid = D_GAIN * (err_pos - prev_err_pos);
  prev_err_pos = err_pos;
  if (i_pid > P_PID_LIMIT) {
    i_pid = P_PID_LIMIT;
  }
  if (i_pid < 0) {
    i_pid = 0;
  }
  cmd_pos = p_pid + i_pid + d_pid;
  if (cmd_pos < DB) {
    cmd_pos = DB;
  }
  else if (cmd_pos > 4095) {
    cmd_pos = 4095;
  }
  dac.setVoltage(cmd_pos, false);
//  if (counter++ == 4095) {
//    counter = 0;
//  }
//  dac.setVoltage(counter, false);
//  Serial.println(current_pos);
  Serial.print(err_pos);
  Serial.print(',');
  Serial.println(desired_pos);
}

void loop() {
  pressure0 = analogRead(PRESSURE0);
  pos0 = analogRead(POS0);
  control_timer.update();
}
