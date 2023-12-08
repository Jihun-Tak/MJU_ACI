#include <stdlib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>


WiFiClient myTCPClient;
PubSubClient myMQTTClient;


int v = 0;
int a = 0;

char s1;
unsigned long int change;
void cbFunc(const char topic[],byte *data, unsigned int length){
  char str[80] = {0,};
  memcpy(str, data, length);

  if (strcmp(topic, "MJU/CD4/CHARGING") == 0) {
    if (strcmp(str, "READY2") == 0) {
    change = 1;
    }
    else if(strcmp(str, "FINISH")==0){
    change = 0;
    }
    else if(strcmp(str, "CHARGINGSTART")==0){
    change = 2;
    Serial.print("aa : ");
    Serial.println(change);
    Serial2.println(change);
    }
    else if(strcmp(str, "CHARGINGSTOP")==0){
    change = 3;
    Serial.println(change);
    Serial2.println(change);
    }
  }
    if (strcmp(topic, "MJU/CD4/CHARGING/CAR") == 0) {
    
    if (strcmp(str, "PutConnector") == 0) { //차량 진입->해당위치로 이동 단계
        a = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  //Serial1.begin(115200,SERIAL_8N1,2,4); //2번핀 RX
  //pinMode(33,INPUT);
  Serial1.begin(115200, SERIAL_8N1, 16, 4);   // 배터리 센싱
  Serial2.begin(115200, SERIAL_8N1, 13, 12);  // 릴레이 동작
  Serial.print("Serial start\n");
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin("MJU_WIFI","mjuwlan!");
  while(1)
  {
    if(WiFi.waitForConnectResult()==WL_CONNECTED){
      printf ("WiFI Connected!\r\n"); break;}

    else{printf("Something Wrong\r\n"); while(1) sleep(1);}
  }
  //Wire.begin(0x13);
  //Wire.onReceive(receiveEvent);

  myMQTTClient.setClient(myTCPClient);
  myMQTTClient.setServer("cheese.mju.ac.kr",30220);
  myMQTTClient.setCallback(cbFunc);
  int result = myMQTTClient.connect("CD4CARBBATESP");
  myMQTTClient.subscribe("MJU/CD4/CHARGING");
  myMQTTClient.subscribe("MJU/CD4/CHARGING/CAR");
  printf("MQTT Conn.. Result:%d\r\n",result);
}



void loop() {
  if(Serial1.available()){
    int sence = Serial1.parseInt();
    if(sence != 0){
      char s1[10];
      Serial.print(sence);
      Serial.println("%");
      snprintf(s1,sizeof(s1), "%d", sence);
      myMQTTClient.publish("MJU/CD4/CHARGING/PER",s1);
    }
  }
  myMQTTClient.loop();
  /*if(Serial1.available()>0){
    v = Serial1.parseInt();
    Serial.print(v);
    Serial.println("connect");
      a++;
      Serial.println(a);
  }*/
  //int signal = analogRead(33);
  //Serial.println(signal);

  if(a == 1){
    Serial.println("ss");
    myMQTTClient.publish("MJU/CD4/CHARGING","READY2");
    delay(300);
    myMQTTClient.publish("MJU/CD4/CHARGING/CAR","Success");
  }

  if(Serial.available()){
    int data = Serial.parseInt();
    if(data == 2) Serial2.print(data);
    else if(data == 3) Serial2.print(data);
  }
}

/*void receiveEvent(int parameter){
  while(Wire.available()){

    if(change == 0){Serial.print("not ready");}
    else if(change == 2){
      Serial.print("START -> ");
     // Serial2.println(47);
    }
        int a = Wire.read();
    char s1[10];
    Serial.print(a);
    Serial.println("%");
    snprintf(s1,sizeof(s1), "%d", a);
    myMQTTClient.publish("MJU/CD4/CHARGING/PER",s1);
  }
}*/