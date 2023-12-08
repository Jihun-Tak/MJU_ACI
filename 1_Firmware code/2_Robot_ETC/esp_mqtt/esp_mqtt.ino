#include <stdlib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

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

void MQTT_START(){  //MQTT Start
  myMQTTClient.setClient(myTCPClient);
  myMQTTClient.setServer(mqttSV,mqttPN);
  myMQTTClient.setCallback(cbFunc);
  int result = myMQTTClient.connect(mqttTSNM);
  myMQTTClient.subscribe(mqttTPz);
  myMQTTClient.subscribe(mqttTPx);
  myMQTTClient.subscribe(mqttTPy);

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