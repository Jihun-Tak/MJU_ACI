#include <SoftwareSerial.h>

#define STEP_DELAY 700

#define DIR_PIN 8
#define STEP_PIN 7
#define ENA_PIN 4

#define motorpin1 5
#define motorpin2 6
#define motorpin3 10
#define motorpin4 11

SoftwareSerial mySerial(13,12);

float wheel_dia = 322; //102.44;
float distance_per_pulse = wheel_dia / 360;

const byte interruptPin1 = 2; //left
const byte interruptPin2 = 3; //right
unsigned long count1 = 0;
unsigned long count2 = 0;
int prev_rot_count1 = 0;
int prev_rot_count2 = 0;
int target = 0;
int target_pulse = 0;
int target_pulse1 = 0;
int target_pulse2 = 0;
int left_flag = 0;
int right_flag = 0;
int start_flag = 0;

int target_value = 0;
int current_value = 0;
int temp = 0;
int motor_speed = 85;

int turn_factor = 230;

void setup()
{
  Serial.begin(115200);
  //mySerial.begin(115200);
  motor_setup();
  attachInterrupt(digitalPinToInterrupt(interruptPin1),intfunc1,RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2),intfunc2,RISING);
}

void loop()
{
  /*
  if ( mySerial.available() )
  {
    target_value = mySerial.parseInt();
    Serial.print("Input data = ");
    Serial.println(target_value);
    if(target_value != 0){
      if(target_value == 12345) target_value = 0;
      if(target_value >= -5000 && target_value <= 5000) step_move();
      if(target_value == 10001) {
        dc_forward(); 
        delay(1200);
        dc_stop();
      }
      if(target_value == 10002) {
        dc_backward(); 
        delay(1200);
        dc_stop();
      }
      if(target_value == 10003) {
        dc_left(); 
        delay(1200);
        dc_stop();
      }
      if(target_value == 10004) {
        dc_right(); 
        delay(1200);
        dc_stop();
      }
      if(target_value == 10005) {
        dc_stop(); 
        delay(1200);
      }
      delay(10);
    }
    digitalWrite(ENA_PIN, HIGH);
  }

  Serial.print("TARGET : ");
  Serial.print(target_value);
  Serial.print(" , CUR : ");
  Serial.print(current_value);
  Serial.println("");
  */

  if(Serial.available()){
    dc_stop();
    Serial.println("Please input data!");
    target = Serial.parseInt();
    if(target != 0){
      if(target > 0) start_flag = 0;
      else if(target < 0) start_flag = -1;

      target = abs(target);      
      Serial.print("Input data : "); Serial.println(target);
      target_pulse1 = (target / distance_per_pulse) - 10;
      target_pulse2 = (target / distance_per_pulse) - 10;
      Serial.print("Target rotation count : "); Serial.println(target_pulse1+10);
      delay(1000);

      if(target == 12345) { //left
        start_flag = 2;
        target_pulse1 = 230;
        target_pulse2 = 230;
      }
      if(target == 23456) { //right
        start_flag = 3;
        target_pulse1 = 230;
        target_pulse2 = 230;//(turn_factor / distance_per_pulse) - 10;
      }

      while(1){
        if(start_flag == 0) {
          dc_forward();
          start_flag = 1;
        }
        else if(start_flag == -1){
          dc_backward();
          start_flag = 1;
        }
        else if(start_flag == 2){
          dc_left();
          start_flag = 1;
        }
        else if(start_flag == 3){
          dc_right();
          start_flag = 1;
        }

        if(prev_rot_count1 != count1) prev_rot_count1 = count1;
        if(prev_rot_count2 != count2) prev_rot_count2 = count2;

        if(left_flag == 0 || right_flag == 0){
          Serial.print("Left count : "); Serial.print(prev_rot_count1);
          Serial.print("  Right count : "); Serial.println(prev_rot_count2);
        }

        if(prev_rot_count1 == target_pulse1){
          left_stop();
          left_flag = 1;
        }
        if(prev_rot_count2 == target_pulse2){
          right_stop();
          right_flag = 1;
        }

        if(left_flag == 1 && right_flag == 1){
          resetencode();
          break;
        }
      }
      
    }
  }
}

void resetencode(){
  Serial.println("Sensor Reset!");
  count1 = 0;
  count2 = 0;
  left_flag = 0;
  right_flag = 0;
  start_flag = 0;
  prev_rot_count1 = 0;
  prev_rot_count2 = 0;
}

void intfunc1(){
  count1++;
}

void intfunc2(){
  count2++;
}

void motor_setup(){
  pinMode(interruptPin1, INPUT_PULLUP);
  pinMode(interruptPin2, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(ENA_PIN, HIGH);
  pinMode(motorpin1, OUTPUT);
  pinMode(motorpin2, OUTPUT);
  pinMode(motorpin3, OUTPUT);
  pinMode(motorpin4, OUTPUT);
}

void step_move(){
    while ( target_value > current_value )
    {
      step_forward();
    }
    while ( target_value < current_value )
    {
      step_backward();
    }
}

void step_forward()
{
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY);
  current_value++;
}

void step_backward()
{
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY);
  current_value--;
}

void left_stop(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, 0);
  Serial.println("Left motor stop");
}

void right_stop(){
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, 0);
  Serial.println("Right motor stop");
}

void dc_forward(){
  analogWrite(motorpin1, motor_speed);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, motor_speed);
  analogWrite(motorpin4, 0);
  Serial.println("Forward");
}

void dc_backward(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, motor_speed);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, motor_speed);
  Serial.println("Backward");

}

void dc_left(){
  analogWrite(motorpin1, motor_speed);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, motor_speed);
  Serial.println("left");
}

void dc_right(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, motor_speed);
  analogWrite(motorpin3, motor_speed);
  analogWrite(motorpin4, 0);
  Serial.println("right");
}


void dc_stop(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, 0);
  Serial.println("stop");
}