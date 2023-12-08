#include <stdlib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

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
    Serial1.println(20000);
    Serial.println("start");
    //neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,RGB_BRIGHTNESS); // Blue
    }
    else if(strcmp(str, "CHARGINGSTOP")==0){
    change = 3;
    Serial1.println(30000);
    Serial.println("stop");
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
  Serial1.begin(115200, SERIAL_8N1, 16, 4);

  Serial.print("start");
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin("MJU_WIFI","mjuwlan!");

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

  if(Serial.available()>0){
    int v = Serial.parseInt();
    Serial1.println(v);
  }


  if(Serial1.available()>0){
    int sense = Serial1.parseInt();
    Serial.println(sense);
    if(sense == 10000){
        myMQTTClient.publish("MJU/CD4/CHARGING/CAR","IN");
    }


  
}
myMQTTClient.loop();
}