#include <stdlib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

int SystemAllFlag =0;
int SystemDCmove = 0;
int SystemFlag = 99;
char mqttSysStep[] = "MJU/CD4/CHARGING/CAR";  //시스템 스텝

int step_target = 0;
int servo_target = 0;
int dc_target = 0;

char ssid[] = "tjh";                              //WiFi ssid
char psw[] = "tak123412341234";                   //Wifi password
char mqttSV[] = "cheese.mju.ac.kr";               //MQTT data
uint16_t mqttPN = 30220;                          //MQTT port number
char mqttTSNM[] = "TESTjaemin01";                 //MQTT test connect
char mqttTPz[] = "MJU/IOT/60181793/Z";  //스텝
char mqttTPy[] = "MJU/IOT/60181793/Y";  //DC
char mqttTPx[] = "MJU/IOT/60181793/X";  //서보

unsigned long long lastMS = 0;
unsigned long long lastMS2 = 0;

int xdata = 0;
int ydata = 0;
int zdata = 0;

int flagz = 0;
int flagy = 0;
int flagx = 0;

WiFiClient myTCPClient;
PubSubClient myMQTTClient;

void cbFunc(const char topic[], byte *data, unsigned int length){ //Callback Functiion
  char DATA[20] = {0,};     //For callback function
  memcpy(DATA, data, length);
  String buffz;
  String buffy;
  String buffx;

  if(topic[17] == 'Z') {
    buffz = DATA;
    zdata = buffz.toInt();
  }
  if(topic[17] == 'Y'){
    buffy = DATA;
    ydata = buffy.toInt() + 20000;
  }
  if(topic[17] == 'X'){
    buffx = DATA;
    xdata = buffx.toInt();
  }

  if (strcmp(topic, "MJU/CD4/CHARGING/CAR") == 0) {
    if (strcmp(str, "IN") == 0) { //차량 진입->해당위치로 이동 단계
        SystemAllFlag = 1;
        SystemDCmove = 21200; //커넥터 위치로 이동 값 (여유있게 줘서 가다가 수발광에 멈추도록?)
        SystemFlag = 0;
    }
    else if(strcmp(str, "FindCARNUM")==0){//번호판 인식단계
        SystemAllFlag = 2;
    }
    else if(strcmp(str, "FindConnector")==0){//커넥터 찾아서 잡는 단계
        SystemAllFlag = 3;
    }
    else if(strcmp(str, "GoToPort")==0){//차량 충전구로 이동 단계
        SystemAllFlag = 4;
        SystemDCmove = 20800; //커넥터 위치로 이동 값 (여유있게 줘서 가다가 수발광에 멈추도록?)
        SystemFlag = 0;
    }
    else if(strcmp(str, "FindPort")==0){//충전구 탐색 및 삽입 단계
        SystemAllFlag = 5;
    }
    else if(strcmp(str, "Success")==0){//충전연결 성공, 커넥터 꽂은채로 로봇팔 회수 단계
        SystemAllFlag = 6;
    }
    else if(strcmp(str, "BackForWaiting")==0){//로봇 원래자리로 돌아가는 단계
        SystemAllFlag = 7;
        SystemDCmove = 23800; //커넥터 위치로 이동 값 (여유있게 줘서 가다가 수발광에 멈추도록?)
        SystemFlag = 0;
    }
    else if(strcmp(str, "WaitingCharge")==0){//충전완료까지 대기하는 단계
        SystemAllFlag = 8;
    }
    else if(strcmp(str, "StopCharging")==0){//충전정지신호 받고 로봇 충전구로 이동하는 단계
        SystemAllFlag = 9;
        SystemDCmove = 20800; //커넥터 (여유있게 줘서 가다가 수발광에 멈추도록?)
        SystemFlag = 0;
    }
    else if(strcmp(str, "PullConnector")==0){//커넥터 찾아서 잡고 전자서 켜서 뽑는 단계
        SystemAllFlag = 10;
    }
    else if(strcmp(str, "PutConnector")==0){//커넥터 돌려놓으면서 회수시스템 작동 단계
        SystemAllFlag = 11;
        SystemDCmove = 23800; //커넥터 위치로 이동 값 (여유있게 줘서 가다가 수발광에 멈추도록?)
        SystemFlag = 0;
    }
    else if(strcmp(str, "CARINWaiting")==0){//원점으로 돌아가는 단계 끝나면 SystemAllFlag = 0으로 돌아가서 반복
        SystemAllFlag = 12;
        SystemDCmove = 23800; //커넥터 위치로 이동 값 (여유있게 줘서 가다가 수발광에 멈추도록?)
        SystemFlag = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 15, 2);
  Serial2.begin(115200, SERIAL_8N1, 12, 13);
  WiFi_START();
  MQTT_START();
}

void loop() {
  myMQTTClient.loop();

  if(Serial2.available()>0){ // 우노가 수발광신호 받으면 다음스텝넘어가도록 플레그 쏴줌.
    SystemFlag = Serial2.parseInt();
  }
    
    if(SystemAllFlag == 1){ //차량 진입->해당충전기 커넥터 위치로 이동 단계
        if(SystemFlag)
        Serial2.print(SystemDCmove);  
    }
    else if(SystemAllFlag == 4){ //차량 충전구로 이동 단계
        if(SystemFlag == 0){ //모서리까지 움직이기
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 27777;
        }
        else if(SystemFlag == 1){// 코너 좌회전
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 20800;
        }
        else if(SystemFlag == 2){ // 충전구까지 이동
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
        }
    }
    else if(SystemAllFlag == 6){ //충전연결 성공, 커넥터 꽂은채로 로봇팔 회수 단계

    }
    else if(SystemAllFlag == 7){ //로봇 원래자리로 돌아가는 단계
        if(SystemFlag == 0){ //모서리까지 움직이기
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 28888;
        }
        else if(SystemFlag == 1){// 코너 우회전
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 24800;
        }
        else if(SystemFlag == 2){ // 커넥터위치
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 25000;
        }
        else if(SystemFlag == 3){ // 원점
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
        }
    }
    else if(SystemAllFlag == 9){ //충전정지신호 받고 로봇 충전구로 이동하는 단계
        if(SystemFlag == 0){
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
            StepDC
        }
        else if(SystemFlag == 1){ //모서리까지 움직이기
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 27777;
        }
        else if(SystemFlag == 1){// 코너 좌회전
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 20800;
        }
        else if(SystemFlag == 2){ // 충전구까지 이동
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
        }
    }
    else if(SystemAllFlag == 11){ //커넥터 돌려놓으면서 회수시스템 작동 단계
        if(SystemFlag == 0){ //모서리까지 움직이기
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 28888;
        }
        else if(SystemFlag == 1){// 코너 우회전
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 24800;
        }
        else if(SystemFlag == 2){ // 커넥터까지 이동
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
        }
        else if(SystemFlag == 3){
            // 서보로 뻗고 스텝으로 내려놓기
        }
    }
    else if(SystemAllFlag == 12){ //원점으로 돌아가는 단계 끝나면 SystemAllFlag = 0으로 돌아가서 반복
        if(SystemFlag == 0){ //모서리까지 움직이기
            Serial2.print(SystemDCmove);
            SystemFlag = 99;
        }
    }

    else if((SystemAllFlag == 3)||(SystemAllFlag == 5)||(SystemAllFlag ==10)){ // 커넥터 잡기 or 충전구 삽입
        if(millis() - lastMS > 2000){
            lastMS = millis();
            if(zdata <=500 && zdata >= 100 && flagz == 0){
            Serial2.print(zdata); 
            Serial.print("z axis data : "); Serial.println(zdata);
            }
            else if(zdata == 9999 && flagz == 0){
            Serial2.print(zdata);
            Serial.print("z axis data : "); Serial.println(zdata);
            flagz = 1;
            }
            if(ydata > 20000 && ydata < 20600 && flagy == 0){       
            Serial2.print(ydata);
            Serial.print("y axis data : "); Serial.println(ydata);
            }
            else if(ydata == 119999 && flagy == 0){
            Serial.print("y axis data : "); Serial.println(ydata);
            flagy = 1;
            }
            if(flagx == 0 && xdata > 50 && xdata < 1000){
            Serial.print("x axis data : "); Serial.println(xdata);
            Serial1.print(xdata);
            flagx = 1;
            }
        }
    }
}


void MQTT_START(){  //MQTT Start
  myMQTTClient.setClient(myTCPClient);
  myMQTTClient.setServer(mqttSV,mqttPN);
  myMQTTClient.setCallback(cbFunc);
  int result = myMQTTClient.connect(mqttTSNM);
  myMQTTClient.subscribe(mqttTPz);
  myMQTTClient.subscribe(mqttTPx);
  myMQTTClient.subscribe(mqttTPy);
  myMQTTClient.subscribe(mqttSysStep);
  
  printf("MQTT Connected. Result : %d\r\n",result);
}

void WiFi_START(){  //WiFi Start
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(ssid,psw);

  while(1){
    if(WiFi.waitForConnectResult() == WL_CONNECTED){
      printf("Connected!\r\n");
      break;
    }
    else{
      printf("Something Wrong\r\n");
      while(1) sleep(1);
    }
  }
}