#include <stdlib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <HardwareSerial.h>

WiFiClient myTCPClient;
PubSubClient myMQTTClient;

char s1;
unsigned long int change;
unsigned long int cable;
int carsensor;
int car;
void cbFunc(const char topic[],byte *data, unsigned int length){
  char str[80] = {0,};
  memcpy(str, data, length);

  if (strcmp(topic, "MJU/CD4/CHARGING") == 0) {
    if (strcmp(str, "READY2") == 0) {
    change = 1;
    digitalWrite(4,LOW);
    }
    else if(strcmp(str, "FINISH")==0){
    change = 0;
    digitalWrite(4,LOW);
    }
    else if(strcmp(str, "CHARGINGSTART")==0){
    change = 2;
    digitalWrite(4,HIGH);
    //neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,RGB_BRIGHTNESS); // Blue
    }
    else if(strcmp(str, "CHARGINGSTOP")==0){
    change = 3;
    digitalWrite(4,LOW);
    //neopixelWrite(RGB_BUILTIN,0,0,0); // Off / black
    }
    else if(strcmp(str,"DCFRONT")==0){
    cable = 1;
    }
    else if(strcmp(str,"DCBACK") == 0){
      cable = 2;
    }
    else if(strcmp(str, "DCSTOP")==0){
      cable = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  //Serial1.begin(115200, SERIAL_8N1, 2, 7);
  Serial.print("start");
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin("MJU_WIFI","mjuwlan!");
  pinMode(4,OUTPUT); //스위치
  pinMode(5,OUTPUT); //회수시스템
  pinMode(6,OUTPUT);
  pinMode(3,OUTPUT); //차량출입센서
  pinMode(2,OUTPUT); // 충전신호

  
  digitalWrite(4,LOW);
  while(1)
  {
    if(WiFi.waitForConnectResult()==WL_CONNECTED){
      printf ("Connected!\r\n"); break;}

    else{printf("Something Wrong\r\n"); while(1) sleep(1);}
  }
  myMQTTClient.setClient(myTCPClient);
  myMQTTClient.setServer("cheese.mju.ac.kr",30220);
  myMQTTClient.setCallback(cbFunc);
  int result = myMQTTClient.connect("60181793mh");
  myMQTTClient.subscribe("MJU/CD4/CHARGING");
  printf("MQTT Conn.. Result:%d\r\n",result);
}

unsigned long long lastMs;

void loop() {
  
  digitalWrite(2,HIGH);
  carsensor = map(analogRead(3), 0, 1023, 0, 5000);
  int i = 3;
  Serial1.println(i);

  if(millis()-lastMs >= 1000)
  { 
    
    lastMs=millis();
    if(cable == 0){
      analogWrite(5, 0);
      analogWrite(6, 0);
      Serial.printf("STOP\n");
      cable = 4;
    }
    else if(cable == 1){
      analogWrite(5, 255/5);
      analogWrite(6, 0);
      Serial.printf("front\n");
      cable = 4;
    }
    else if(cable == 2){
      analogWrite(5, 0);
      analogWrite(6,255/5);
      Serial.printf("back\n");
      cable = 4;
    }
    if(carsensor >= 10000){
      car++;
    }
    else if(carsensor < 10000){
      car = 0;
    }
    Serial.println(carsensor);
    Serial.println(car);

    if(car == 3){
      myMQTTClient.publish("MJU/CD4/CHARGING/CAR","IN");
    }
  }
  



  myMQTTClient.loop();
}