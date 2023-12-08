/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>
#include <stdlib.h>
#include <math.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 

const uint8_t DXL_1 = 100; // 서보모터 이름 / 반이중 UART로 모든 모터가 연결되어 있어 
const uint8_t DXL_2 = 75;  // 각 서보에 이름을 부여하여 해당 이름을 함께 데이터 패킷에 묶어서 데이터 전송
const uint8_t DXL_3 = 50;
const uint8_t DXL_4 = 25;

float servo = 35.25;       // 각 링크를 계산하기 위한 각 파트별 길이
float short_servo = 14.25;
float long_joint = 55.0;
float short_joint = 28;

float link0 = servo;                      // 원점 -> 조인트 1 까지의 거리
float link1 = long_joint + servo;         // 조인트1 -> 조인트2 까지의 거리
float link2 = long_joint + servo;         // 조인트2 -> 조인트3 까지의 거리
float link3 = short_joint + short_servo;  // 조인트3 -> 엔드이펙터 까지의 거리
float y_min = 60;
int temp = 60;
int servo_flag = 0;
int temp_x = 0;
int temp_y = 0;
int rotate_flag = 0;

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_1);
  dxl.ping(DXL_2);
  dxl.ping(DXL_3);

  dxl.torqueOff(DXL_1);
  dxl.setOperatingMode(DXL_1, OP_POSITION);
  dxl.torqueOn(DXL_1);

  dxl.torqueOff(DXL_2);
  dxl.setOperatingMode(DXL_2, OP_POSITION);
  dxl.torqueOn(DXL_2);

  dxl.torqueOff(DXL_3);
  dxl.setOperatingMode(DXL_3, OP_POSITION);
  dxl.torqueOn(DXL_3);

  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_1, 120);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_2, 120);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_3, 120);

  moving_servo(60);
}

void moving_servo(int data){                  // 로봇 팔의 움직임을 부드럽게 만들기 위한 코드
  int target = data;
  Serial.print("target : "); Serial.println(target);
  Serial.print("temp : "); Serial.println(temp);  
  if(target > temp){                          // 목표 위치가 기존 위치보다 멀 경우
    for(int a = temp ; a < target+1 ; a ++){  // 서보모터가 급격하게 움직이는것을 방지하기 위해 기존 위치부터 목표 위치까지
      Serial.print(a); Serial.print(" ");     // 1mm씩 순차적으로 움직이도록 코드로써 구현
      InverseKinematics(0,a);
    }
    Serial.println(); Serial.println();
  }
  else{
    for(int a = temp ; a > target-1 ; a --){
      Serial.print(a); Serial.print(" "); 
      InverseKinematics(0,a);
    }
    Serial.println(); Serial.println();
  }
  temp = target;
}

void InverseKinematics(float x_data, float y_data){
  float x = x_data;   // End-effector 목표 X 좌표
  float y = y_data;   // End-effector 목표 Y 좌표
  float cos_theta2 = (x*x+y*y-link1*link1-link2*link2)/(2*link1*link2); // 역 기구학 계산
  float sin_theta2 = sqrt(1-cos_theta2*cos_theta2);

  float theta1 = degrees(atan2(y,x)-atan2(link2*sin_theta2,link1+link2*cos_theta2))+90;   // 1번 서보모터 각도
  float theta2 = degrees(atan2(sin_theta2, cos_theta2))+180;                              // 2번 서보모터 각도
  float theta3 = 540 - theta1 - theta2;                                                   // 3번 서보모터 각도

  if(servo_flag == 1) ;               // 로봇팔 목표가 중앙일 경우 그대로
  if(servo_flag == 2) theta1 += 90;   // 로봇팔 목표가 좌측일 경우 1번 서보모터를 90도 돌림
  if(servo_flag == 3) {               // 로봇팔 목표가 우측일 경우 재계산
    theta1 -= 90;
    if(theta1 <= 90){
      theta1 = 180 - theta1;
      theta2 = 360 - theta2;
      theta3 = 450 - theta1 - theta2;
    }
  }
  dxl.setGoalPosition(DXL_1, theta1, UNIT_DEGREE);    // 서보모터가 목표 각도로 회전하도록 데이터 패킷을 조립해 UART 출력하는 함수
  dxl.setGoalPosition(DXL_2, theta2, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_3, theta3, UNIT_DEGREE);
}

<<<<<<< HEAD
void loop() { ㅁ
=======
void loop() { 
>>>>>>> 4e9d6364379416d09e70a15f483b89b791a6994d
  if(Serial2.available()){
    int target = Serial2.parseInt();
    if( target > 0 && target < 1000){ // 로봇팔 중앙으로
      if(rotate_flag == 2){  // 왼쪽에서 중앙으로
        moving_servo(60);    // 왼쪽에서 끝까지 당김
      }
      if(rotate_flag == 3){  // 오른쪽에서 중앙으로
        moving_servo(60);    // 오른쪽에서 끝까지 당김
        delay(100);
        servo_flag = 2;
        moving_servo(60);    // 왼쪽으로 회전
        delay(100);
        servo_flag = 1;
        moving_servo(60);    // 중앙으로 회전 / 그런데 딜레이가 짧아서 왼쪽으로 회전하다가 중간에 중앙으로 회전함.
      }                      // 원래 오른쪽에서 중앙으로 바로 회전하는데 그러면 모터 속도차이 때문에 2번 모터가 바닥을 때리는데, 왠지 모르겠는데 오른쪽에서 왼쪽으로 돌리면 바닥을 안때리고 
      servo_flag = 1;
      Serial.print("target : "); Serial.println(target);
      if(rotate_flag == 2 || rotate_flag == 3){
        moving_servo(60);
      }
      moving_servo(target);
      rotate_flag = 1;
    }
    else if(target > 1000 && target < 2000){ // 로봇 팔 왼쪽으로
      if(rotate_flag == 1 || rotate_flag == 3){
        moving_servo(60);
      }
      servo_flag = 2;
      target -= 1000;
      if(rotate_flag == 1 || rotate_flag == 3){
        moving_servo(60);
      }
      moving_servo(target);
      rotate_flag = 2;
    }
    else if(target > 2000 && target < 3000){ // 로봇 팔 오른쪽으로
      if(rotate_flag == 1 || rotate_flag == 2){
        moving_servo(60);
      }
      servo_flag = 3;
      target -= 2000;
      if(rotate_flag == 1 || rotate_flag == 2){
        moving_servo(60);
      }
      moving_servo(target);
      rotate_flag = 3;
    }
  }
}