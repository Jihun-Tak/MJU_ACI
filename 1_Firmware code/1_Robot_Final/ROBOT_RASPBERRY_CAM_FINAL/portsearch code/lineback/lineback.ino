#include <stdio.h>
#include <stdlib.h>
#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient myTCPClient;
PubSubClient myMQTTClient;

int change =0;

void cbFunc(const char topic[],byte *data, unsigned int length){
  char str[80] = {0,};
  memcpy(str, data, length);

  if (strcmp(topic, "MJU/CD4/CHARGING") == 0) {
    if (strcmp(str, "GRAB") == 0) {
    change = 1;
    } 
    else if (strcmp(str, "END") == 0) {
      change = 2;
    }
    else if (strcmp(str, "STOP") == 0) {
      change = 0;
    }
  }
}

void setup() {

  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin("MJU_WIFI","mjuwlan!");
  while(1)
  {
    if(WiFi.waitForConnectResult()==WL_CONNECTED){
      printf ("Connected!\r\n"); break;}

    else{printf("Something Wrong\r\n"); while(1) sleep(1);}
  }

  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);

  myMQTTClient.setClient(myTCPClient);
  myMQTTClient.setServer("cheese.mju.ac.kr",30220);
  myMQTTClient.setCallback(cbFunc);
  int result = myMQTTClient.connect("60181793mh");
  myMQTTClient.subscribe("MJU/CD4/CHARGING");
  printf("MQTT Conn.. Result:%d\r\n",result);

}
unsigned long long lastMs;

void loop() {
  // put your main code here, to run repeatedly:
  //int inputValue = analogRead(A0);
  //int convertedValue = map(inputValue, 0, 1023, 0, 255);
  
  //if(센서에 값 들어오면)

  
  if(millis()-lastMs >= 1000)
  {
    lastMs=millis();
    if(change == 0){
      digitalWrite(2, LOW);
      digitalWrite(4, LOW);
      Serial.printf("STOP\n");
      change = 4;
    }
    else if(change == 1){
      digitalWrite(2, HIGH);
      digitalWrite(4, LOW);
      Serial.printf("front\n");
      change = 4;
    }
    else if(change == 2){
      digitalWrite(2,LOW);
      digitalWrite(4,HIGH);
      Serial.printf("back\n");
      change = 4;
    }
    else if(change == 4){
      Serial.printf("empty\n");
    }
    
  }
  myMQTTClient.loop();
}
