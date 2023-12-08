#include <stdlib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

int current = 0;
int SystemAllFlag =0;
int SystemDCmove = 0;
int SystemFlag = 99;
int Chargingflag = 0;
int ChargingStart = 0;
char mqttSysStep[] = "MJU/CD4/CHARGING/CAR";  //시스템 스텝

int step_target = 0;
int servo_target = 0;
int dc_target = 0;

char ssid[] = "MJU_WIFI";                              //WiFi ssid
char psw[] = "mjuwlan!";                   //Wifi password
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


//커넥터 높이, 충전구 높이
int connector_step = 10070; // 커넥터 스텝높이
int connector_servo = 2135; // 커넥터 서보깊이
int port_step = 10250; // 충전구 스텝높이
int port_servo = 1117;

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
    //Serial2.println(zdata);
  }
  if(topic[17] == 'Y'){
    int B;
    buffy = DATA;
    /*int b = 0;
    b = buffy.toInt();
    Serial.print("Y axis data input :"); Serial.println(b);
    if(b > 354){
      b = abs(354-b);
      b *= 0.4347826087;
      b += 20000;
      Serial.print("Go front : "); Serial.println(b);
      ydata = b;
    }*/
    B = buffy.toInt();
    B = 354 - B;
    Serial.print("Y data : ");
    Serial.println(B);
    if(B>0){
      ydata = (B*0.4347826087) + 23000;
    }
    else if(B<0){
      B=abs(B);
      ydata = (B*0.4347826087) + 20000;
    }
    Serial.println(ydata);
    //Serial2.println(ydata);
  }
  if(topic[17] == 'X'){           // 서보 중앙   : 60 ~ 170
    buffx = DATA;                 // 서보 왼쪽   : 1060 ~ 1170
    xdata = buffx.toInt();        // 서보 오른쪽 : 2060 ~ 2170
    //Serial1.println(xdata);
  }

  if (strcmp(topic, "MJU/CD4/CHARGING") == 0){
    if(strcmp(DATA, "CHARGINGSTART")==0){
      ChargingStart = 1;
    }
    else if(strcmp(DATA, "CHARGINGSTOP")==0){
      ChargingStart = 0;
      Serial2.println(13000); //모터스피드100
      SystemAllFlag = 9;
      current = 0;
      SystemDCmove = 21000; //커넥터 (여유있게 줘서 가다가 수발광에 멈추도록?)
      SystemFlag = 0;
      Serial.println("StopCharging");
    }
  }

  if (strcmp(topic, "MJU/CD4/CHARGING/CAR") == 0) {
    Serial.println(DATA);
    
    if (strcmp(DATA, "IN") == 0) { //차량 진입->해당위치로 이동 단계
        Serial2.println(13000); //모터스피드100
        SystemAllFlag = 1;
        SystemDCmove = 21500; //커넥터 위치로 이동 값 (여유있게 줘서 가다가 수발광에 멈추도록?)
        SystemFlag = 0;
        current = 0;
        Serial.println("IN");
    }
    else if(strcmp(DATA, "FindCARNUM")==0){//번호판 인식단계
    
        SystemAllFlag = 2;
        SystemFlag = 0;
        current = 0;
        Serial.println("FindCARNUM");
        
    }
    else if(strcmp(DATA, "FindConnector")==0){//커넥터 찾아서 잡는 단계
        SystemAllFlag = 3;
        SystemFlag = 0;
        current = 0;
        Serial.println("FindConnector22");
        Serial2.println(connector_step); //모터스피드 60
    }
    else if(strcmp(DATA, "GoToPort")==0){//차량 충전구로 이동 단계
        Serial2.println(13000); //모터스피드100
        SystemAllFlag = 4;
        current = 0;
        SystemDCmove = 23700; 
        SystemFlag = 0;
        Serial.println("GoToPort");
        
    }
    else if(strcmp(DATA, "FindPort")==0){//충전구 탐색 및 삽입 단계
        SystemAllFlag = 5;
        current = 0;
        Serial.println("FindPort");
        Serial2.println(10180);
        flagz = 0;
        flagy = 0;
        flagx = 0;
    }
    else if(strcmp(DATA, "Success")==0){//충전연결 성공, 커넥터 꽂은채로 로봇팔 회수 단계
        Serial2.println(13000); //모터스피드100
        SystemAllFlag = 6;
        current = 0;
        Serial.println("Success");
        Chargingflag = 1;
    }
    else if(strcmp(DATA, "BackForWaiting")==0){//로봇 원래자리로 돌아가는 단계
        Serial2.println(13000); //모터스피드100
        SystemAllFlag = 7;
        current = 0;
        SystemDCmove = 23600;
        SystemFlag = 0;
        Serial.println("BackForWaiting");
    }
  
    else if(strcmp(DATA, "WaitingCharge")==0){//충전완료까지 대기하는 단계
        SystemAllFlag = 8;
        current = 0;
        Serial.println("WaitingCharge");
        
    }
    else if(strcmp(DATA, "StopCharging")==0){//충전정지신호 받고 로봇 충전구로 이동하는 단계
        Serial2.println(13000); //모터스피드100
        SystemAllFlag = 9;
        current = 0;
        SystemDCmove = 21000; //커넥터 (여유있게 줘서 가다가 수발광에 멈추도록?)
        SystemFlag = 0;
        Serial.println("StopCharging");
    }
    else if(strcmp(DATA, "PullConnector")==0){//커넥터 찾아서 잡고 전자서 켜서 꽂는단계(뽑기위해)
        Serial2.println(10200);
        SystemAllFlag = 10;
        current = 0;
        Serial.println("PullConnector");
        flagz = 0;
        flagy = 0;
        flagx = 0;
    }
    else if(strcmp(DATA, "PutConnector")==0){//커넥터 뽑고 돌려놓으면서 회수시스템 작동 단계
        Serial2.println(15000); //전자석 잡기
        delay(300);
        Serial2.println(13000); //모터스피드100
        SystemAllFlag = 11;
        current = 0;
        SystemDCmove = 23600; //커넥터 위치로 이동 값 (여유있게 줘서 가다가 수발광에 멈추도록?)
        SystemFlag = 0;
        Serial.println("PutConnector");
    }
    else if(strcmp(DATA, "CARINWaiting")==0){//원점으로 돌아가는 단계 끝나면 SystemAllFlag = 0으로 돌아가서 반복
        Serial2.println(13000); //모터스피드100
        SystemAllFlag = 12;
        current = 0;
        SystemDCmove = 23700; //커넥터 위치로 이동 값 (여유있게 줘서 가다가 수발광에 멈추도록?)
        SystemFlag = 0;
        Serial.println("CARINWaiting");
    }
    
    else if(strcmp(DATA, "RESET") == 0){ //전체 시스템 리셋
        SystemAllFlag = 99;
        SystemFlag = 0;
        Serial.println("RESET");
    }
    else if(strcmp(DATA, "TEST_F") == 0){
      Serial2.println(13000);
      delay(200);
      Serial2.println(20300);
    }
    else if(strcmp(DATA, "TEST_B") == 0){
      Serial2.println(13000);
      delay(200);
      Serial2.println(23300);
    }
    else if(strcmp(DATA, "TEST_R") == 0){
      Serial2.println(13000);
      delay(200);
      Serial2.println(28888);
    }
    else if(strcmp(DATA, "TEST_L") == 0){
      Serial2.println(13000);
      delay(200);
      Serial2.println(27777);
    }
    else if(strcmp(DATA, "Systemflag") == 0){
      current += 1;
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

  if(Serial.available()>0){
    int v = Serial.parseInt();
    if(v >10000){
        Serial2.println(v);
    }
    else if(v <=10000){
      Serial1.println(v);
    }
  }

  if(Serial2.available()>0){ // 우노가 수발광신호 받으면 다음스텝넘어가도록 플레그 쏴줌.
    int i = Serial2.parseInt();
    Serial.println(i);
    Serial.print("값:");
    Serial.println(current);
    if(i == 1){
      current += 1;
      SystemFlag = current;
    }
    Serial.println(SystemFlag);
  }
    if(Serial1.available()>0){ // 서보가 다음스텝넘어가도록 플레그 쏴줌.
    int i = Serial1.parseInt();
    if(i == 0){
      current += 1;
      SystemFlag = current;
    }
  }
    
    if(SystemAllFlag == 1){ //차량 진입->해당충전기 커넥터 위치로 이동 단계
        if(SystemFlag == 0){
            delay(300);
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 20600;
            Serial1.println(60);
        }
        else if(SystemFlag == 1){
            delay(300);
            Serial2.println(SystemDCmove);
            delay(500);
            Serial2.println(10170); //번호판 위치
            SystemFlag = 99;
            delay(1000);
            myMQTTClient.publish("MJU/CD4/CHARGING/CAR","FindCARNUM");
            
        }
    }
    
    else if(SystemAllFlag == 2){ // 번호판 촬영
      if(SystemFlag == 0){
        delay(300);
        Serial1.println(2060); //오른쪽 보기
        delay(500);
        Serial2.println(10200); // 스텝 높이 200mm
        SystemFlag = 99;
      }
    }

    else if(SystemAllFlag == 3){//커넥터 잡기
      if(SystemFlag == 0){
        Serial.println("connector");
        delay(4000);//스텝내릴때까지 대기
        Serial1.println(connector_servo);
        delay(2000);
        Serial2.println(15000); //전자석 켜기
        delay(500);
        Serial2.println(10150);
        delay(500);
        Serial1.println(2060);
        myMQTTClient.publish("MJU/CD4/CHARGING/CAR","GoToPort");
        SystemFlag = 99;
      }
    }

    else if(SystemAllFlag == 4){ //차량 충전구로 이동 단계
        if(SystemFlag == 0){ //커넥터 들기 모서리까지 움직이기
            delay(300);
            Serial2.println(13000);
            delay(100);
            Serial1.println(2060);
            delay(1500);
            Serial2.println(SystemDCmove);
            SystemDCmove = 28888;
            SystemFlag = 99;
        }
        else if(SystemFlag == 1){// 코너 우회전
            Serial.print("Flag1 : ");
            Serial.println(SystemFlag);

            delay(300);
            Serial2.println(SystemDCmove);
            delay(100);
            Serial1.println(1060);
            SystemDCmove = 20700;
            
            SystemFlag = 99;
        }
        else if(SystemFlag == 2){
          delay(300);
          Serial2.println(SystemDCmove);
          Serial.print("Flag2 : ");
            Serial.println(SystemFlag);
            SystemFlag = 99;
        }
        else if(SystemFlag == 3){
            Serial.print("Flag3 : ");
            Serial.println(SystemFlag);
            myMQTTClient.publish("MJU/CD4/CHARGING/CAR","FindPort");
        }
    }
    else if(SystemAllFlag == 6){ //충전연결 성공, 커넥터 꽂은채로 로봇팔 회수 단계
        if(ChargingStart == 1){
          delay(300);
          Serial2.println(16000); //전자석 놓기
          delay(100);
          Serial1.println(1060);
          delay(1000);
          Serial2.println(10100);
          myMQTTClient.publish("MJU/CD4/CHARGING/CAR","BackForWaiting");
        }
    }
    else if(SystemAllFlag == 7){ //로봇 원래자리로 돌아가는 단계
        if(SystemFlag == 0){ //모서리까지 움직이기
            delay(300);
            Serial2.println(13000);
            delay(300);
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 27777;
        }
        else if(SystemFlag == 1){// 코너 우회전
            Serial1.println(60);
            delay(300);
            Serial2.println(SystemDCmove);
            SystemDCmove = 23700;
            
        }
        else if(SystemFlag == 2){
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
        }
        else if(SystemFlag == 3){
            myMQTTClient.publish("MJU/CD4/CHARGING/CAR","WaitingCharge");
        }
    }
    else if(SystemAllFlag == 9){ //충전정지신호 받고 로봇 충전구로 이동하는 단계
        if(SystemFlag == 0){
            delay(300);
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 28888;
        }
        else if(SystemFlag == 1){// 코너 우회전
            delay(300);
            Serial2.println(SystemDCmove);
            SystemDCmove = 20700;
            delay(300);
            Serial1.println(1060);  
        }
        else if(SystemFlag == 2){
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
        }
        else if(SystemFlag == 3){
          myMQTTClient.publish("MJU/CD4/CHARGING/CAR","PullConnector");
        }
    }
    else if(SystemAllFlag == 11){ //커넥터 돌려놓으면서 회수시스템 작동 단계
        if(SystemFlag == 0){ //모서리까지 움직이기
            Serial1.println(1060);
            delay(500);
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 27777;
        }
        else if(SystemFlag == 1){// 코너 좌회전
            delay(200);
            Serial2.println(SystemDCmove);
            delay(200);
            Serial1.println(2060);
            SystemFlag = 99;
            SystemDCmove = 20600;
        }
        else if(SystemFlag == 2){
            delay(300);
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
        }
        else if(SystemFlag == 3){
            delay(300);
            SystemFlag = 99;
            Serial1.println(connector_servo);
            delay(1500);
            Serial2.println(connector_step);
            delay(4000);
            Serial2.println(16000);//전자석 놓기
            delay(300);
            Serial1.println(2060);
            delay(1000);
            Serial.println("서보 놓기 완료");
            myMQTTClient.publish("MJU/CD4/CHARGING/CAR","CARINWaiting");
        }

    }
    else if(SystemAllFlag == 12){ //원점으로 돌아가는 단계 끝나면 SystemAllFlag = 0으로 돌아가서 반복
        if(SystemFlag == 0){ //모서리까지 움직이기
            delay(300);
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            delay(500);
            Serial1.println(60);
        }
        else if(SystemFlag == 1){
          delay(300);
          Serial2.println(SystemDCmove);
          SystemFlag = 99;
          delay(300);
          Serial2.println(10050);
        }
    }

    else if((SystemAllFlag == 5)||(SystemAllFlag ==10)){ // 커넥터 잡기 or 충전구 삽입
        if(millis() - lastMS > 5000){
            lastMS = millis();
            if(zdata <=500 && zdata >= 30){
              zdata = zdata * 0.337078651685;
            zdata = zdata + 10000;
            Serial2.println(zdata); 
            Serial.print("z axis data : "); Serial.println(zdata);
            }
            else if(zdata == 9999){
            zdata = zdata + 30000;
            //Serial2.println(zdata);
            Serial.print("z axis data : "); Serial.println(zdata);
            flagz = 1;
            }

            if(ydata > 20000 && ydata < 20600){       
            Serial2.println(ydata);
            Serial.print("y axis data : "); Serial.println(ydata);
            }
            else if(ydata == 99999){
            Serial.print("y axis data : "); Serial.println(ydata);
            flagy = 1;
            }
            if(flagx == 0 && xdata > 50 && xdata < 1000){
            Serial.print("x axis data : "); Serial.println(xdata);
              if(SystemAllFlag == 5){
                xdata = xdata + 1000;
                Serial1.println(port_servo);
                myMQTTClient.publish("MJU/CD4/CHARGING/CAR","Success");
                myMQTTClient.publish("MJU/CD4/CHARGING","READY2");
              }
              else if(SystemAllFlag == 10){
                xdata = xdata + 1000;
                Serial1.println(xdata);
                delay(1500);
                myMQTTClient.publish("MJU/CD4/CHARGING/CAR","PutConnector");
              }
            
            flagx = 1; // 꽂았을때 신호 안들어오면 서보 당긴 후 다시시도.
            }
            
        }
    }
    else if(SystemAllFlag == 99){ // 시스템 초기화하는 코드(uno, 서보모터보드에 리셋 코드 보내주는 단계)
        
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
  myMQTTClient.subscribe("MJU/CD4/CHARGING");
  
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