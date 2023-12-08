#include <stdlib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <HardwareSerial.h>

WiFiClient myTCPClient;
PubSubClient myMQTTClient;

char s1;
unsigned long int change;
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
    neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,RGB_BRIGHTNESS); // Blue
    }
    else if(strcmp(str, "CHARGINGSTOP")==0){
    change = 3;
    digitalWrite(4,LOW);
    neopixelWrite(RGB_BUILTIN,0,0,0); // Off / black
    }
    else if(strcmp(str,"GRAB")==0){

    }
    else if(strcmp(str,"END") == 0){

    }
    else if(strcmp(str, "STOP")==0){
      
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.print("start");
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin("MJU_WIFI","mjuwlan!");
  pinMode(4,OUTPUT);
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

void loop() {
  myMQTTClient.loop();
}