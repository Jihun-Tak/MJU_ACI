#include <stdlib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
// 17 16 4   27 14 12 13   0 2 15 Normaly open 
#define relay_all_1 17  //SMPS
#define relay_all_2 16
#define relay_all_3 4

#define relay_12v_1 27  //병렬
#define relay_12v_2 14
#define relay_12v_3 12
#define relay_12v_4 13

#define relay_96v_1 0   //직렬
#define relay_96v_2 2
#define relay_96v_3 15


char ssid[] = "MJU_WIFI";                              //WiFi ssid
char psw[] = "mjuwlan!";                   //Wifi password
char mqttSV[] = "cheese.mju.ac.kr";               //MQTT data
uint16_t mqttPN = 30220;                          //MQTT port number
char mqttTSNM[] = "CD4CARBATESP";                 //MQTT test connect
char mqttSysStep[] = "MJU/CD4/CHARGING/CAR";  //시스템 스텝

WiFiClient myTCPClient;
PubSubClient myMQTTClient;

void cbFunc(const char topic[], byte *data, unsigned int length){ //Callback Functiion
  char DATA[20] = {0,};     //For callback function
  memcpy(DATA, data, length);
  String buffz;
  String buffy;
  String buffx;
  if(strcmp(topic, "MJU/CD4/CHARGING") == 0){
    if(strcmp(DATA, "CHARGINGSTART") == 0){
      Serial.println("Charging start!");
      relay_charging();
    }
    else if(strcmp(DATA, "StopCharging") == 0){
      Serial.println("Stop charging!");
      relay_operating();
    }
  }
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

void MQTT_START(){  //MQTT Start
  myMQTTClient.setClient(myTCPClient);
  myMQTTClient.setServer(mqttSV,mqttPN);
  myMQTTClient.setCallback(cbFunc);
  int result = myMQTTClient.connect(mqttTSNM);
  myMQTTClient.subscribe(mqttSysStep);
  myMQTTClient.subscribe("MJU/CD4/CHARGING");
  
  printf("MQTT Connected. Result : %d\r\n",result);
}

void relay_setup(){
  pinMode(relay_all_1,OUTPUT);
  pinMode(relay_all_2,OUTPUT);
  pinMode(relay_all_3,OUTPUT);
  pinMode(relay_12v_1,OUTPUT);
  pinMode(relay_12v_2,OUTPUT);
  pinMode(relay_12v_3,OUTPUT);
  pinMode(relay_12v_4,OUTPUT);
  pinMode(relay_96v_1,OUTPUT);
  pinMode(relay_96v_2,OUTPUT);
  pinMode(relay_96v_3,OUTPUT);
}

void relay_initial_mode(){            // SMPS OFF    병렬 ON    직렬 OFF
  delay(10000); // 1. SMPS OFF
  Serial.println("SMPS OFF");
  digitalWrite(relay_all_1,HIGH);    // SMPS    NO   ON LOW
  digitalWrite(relay_all_2,HIGH);    //             OFF HIGH
  digitalWrite(relay_all_3,HIGH);
  delay(10000); // 2. 병렬 OFF
  Serial.println("병렬 OFF");
  digitalWrite(relay_12v_1,LOW);    // 병렬 제어 NC   ON HIGH
  digitalWrite(relay_12v_2,LOW);    //              OFF LOW
  digitalWrite(relay_12v_3,LOW);
  digitalWrite(relay_12v_4,LOW);
  delay(10000); // 3. 직렬 OFF
  Serial.println("직렬 OFF");
  digitalWrite(relay_96v_1,HIGH);    // 직렬 제어 NO  ON LOW
  digitalWrite(relay_96v_2,HIGH);    //             OFF HIGH
  digitalWrite(relay_96v_3,HIGH);
  delay(10000); // 2. 병렬 ON
  Serial.println("병렬 ON");
  digitalWrite(relay_12v_1,LOW);    // 병렬 제어 NC   ON HIGH
  digitalWrite(relay_12v_2,LOW);    //              OFF LOW
  digitalWrite(relay_12v_3,LOW);
  digitalWrite(relay_12v_4,LOW);
}

void relay_charging(){               // SMPS ON    병렬 ON     직렬 OFF
  delay(10000); // 1. SMPS OFF
  Serial.println("SMPS OFF");
  digitalWrite(relay_all_1,HIGH);    // SMPS    NO   ON LOW
  digitalWrite(relay_all_2,HIGH);    //             OFF HIGH
  digitalWrite(relay_all_3,HIGH);
  delay(10000); // 2. 병렬 OFF
  Serial.println("병렬 OFF");
  digitalWrite(relay_12v_1,LOW);    // 병렬 제어 NC   ON HIGH
  digitalWrite(relay_12v_2,LOW);    //              OFF LOW
  digitalWrite(relay_12v_3,LOW);
  digitalWrite(relay_12v_4,LOW);
  delay(10000); // 3. 직렬 OFF
  Serial.println("직렬 OFF");
  digitalWrite(relay_96v_1,HIGH);    // 직렬 제어 NO  ON LOW
  digitalWrite(relay_96v_2,HIGH);    //             OFF HIGH
  digitalWrite(relay_96v_3,HIGH);
  delay(10000); // 4. SMPS ON
  Serial.println("SMPS ON");
  digitalWrite(relay_all_1,LOW);    // SMPS    NO   ON LOW
  digitalWrite(relay_all_2,LOW);    //             OFF HIGH
  digitalWrite(relay_all_3,LOW);
  delay(10000); // 2. 병렬 ON
  Serial.println("병렬 ON");
  digitalWrite(relay_12v_1,HIGH);    // 병렬 제어 NC   ON HIGH
  digitalWrite(relay_12v_2,HIGH);    //              OFF LOW
  digitalWrite(relay_12v_3,HIGH);
  digitalWrite(relay_12v_4,HIGH);
}

void relay_operating(){             // SMPS OFF    병렬 OFF    직렬 ON
  delay(10000); // 1. SMPS OFF
  Serial.println("SMPS OFF");
  digitalWrite(relay_all_1,HIGH);    // SMPS    NO   ON LOW
  digitalWrite(relay_all_2,HIGH);    //             OFF HIGH
  digitalWrite(relay_all_3,HIGH);
  delay(10000); // 2. 병렬 OFF
  Serial.println("병렬 OFF");
  digitalWrite(relay_12v_1,LOW);    // 병렬 제어 NC   ON HIGH
  digitalWrite(relay_12v_2,LOW);    //              OFF LOW
  digitalWrite(relay_12v_3,LOW);
  digitalWrite(relay_12v_4,LOW);
  
  delay(10000); // 3. 직렬 OFF
  Serial.println("직렬 OFF");
  digitalWrite(relay_96v_1,HIGH);    // 직렬 제어 NO  ON LOW
  digitalWrite(relay_96v_2,HIGH);    //             OFF HIGH
  digitalWrite(relay_96v_3,HIGH);
  delay(10000); // 5. 직렬 ON
  Serial.println("직렬 ON");
  digitalWrite(relay_96v_1,LOW);    // 직렬 제어 NO  ON LOW
  digitalWrite(relay_96v_2,LOW);    //             OFF HIGH
  digitalWrite(relay_96v_3,LOW);
}

void setup() {
  Serial.begin(115200);
  WiFi_START();
  MQTT_START();
  relay_setup();
  delay(500);
  relay_initial_mode();
  delay(500);
}

void loop() {
  myMQTTClient.loop();

  if(Serial.available()>0){
    int data = Serial.parseInt();
    if(data == 1){
      Serial.println("SMPS OFF | 병렬 ON | 직렬 OFF");
      relay_initial_mode();
    }
    else if(data == 2){
      Serial.println("SMPS ON | 병렬 ON | 직렬 OFF");
      relay_charging();
    }
    else if(data == 3){
      Serial.println("SMPS OFF | 병렬 OFF | 직렬 ON");
      relay_operating();
    }
  }
}



