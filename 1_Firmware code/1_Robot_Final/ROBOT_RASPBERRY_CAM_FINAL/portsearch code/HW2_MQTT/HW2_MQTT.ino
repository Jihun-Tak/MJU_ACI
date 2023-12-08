#include <stdlib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

int MPU_Address = 0x68; // MPU6050 칩 I2C주소
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
unsigned long int change;
unsigned long int rgb, r,g,b;
WiFiClient myTCPClient;
PubSubClient myMQTTClient;

void cbFunc(const char topic[],byte *data, unsigned int length){
  char str[80] = {0,};
  memcpy(str, data, length);

  if (strcmp(topic, "MJU/IOT/60181793/TYPE") == 0) {
    if (strcmp(str, "ACCEL") == 0) {
    change = 1;
  } 
  else if (strcmp(str, "TEMP") == 0) {
    change = 0;
  }
  } 
  else if (strcmp(topic, "MJU/IOT/60181793/RGB") == 0) {
     sscanf(str, "0x%2X%2X%2X", &r, &g, &b);
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

  Wire.begin(4,5);
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x75);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_Address,1);
  byte whoami = Wire.read();
  Serial.print(whoami);
  if(whoami == 0x68){
    Serial.print("is MPU6050 (0x68)");
  }
  else{
    Serial.print("fail");
  }

  Wire.beginTransmission(MPU_Address);
  Wire.write(0x6B);
  Wire.write(1);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_Address);
  Wire.write(0x1C);
  Wire.write(0x18);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_Address);
  Wire.write(0x1B);
  Wire.write(0x18);
  Wire.endTransmission();

  
  myMQTTClient.setClient(myTCPClient);
  myMQTTClient.setServer("cheese.mju.ac.kr",30220);
  myMQTTClient.setCallback(cbFunc);
  int result = myMQTTClient.connect("60181793mh");
  myMQTTClient.subscribe("MJU/IOT/60181793/RGB");
  myMQTTClient.subscribe("MJU/IOT/60181793/TYPE");
  printf("MQTT Conn.. Result:%d\r\n",result);
}

unsigned long long lastMs;

void loop() {
  // put your main code here, to run repeatedly:

  Wire.beginTransmission(MPU_Address);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(MPU_Address, 14);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  float(valueAcX) = AcX;
  valueAcX = valueAcX / 2048;
  
  float(valueAcY) = AcY;
  valueAcY = valueAcY / 2048;
  
  float(valueAcZ) = AcZ;
  valueAcZ = valueAcZ / 2048;


  Serial.print("AcX = "); Serial.print(valueAcX);
  Serial.print(", AcY = "); Serial.print(valueAcY);
  Serial.print(", AcZ = "); Serial.println(valueAcZ);

  Serial.print("Tmp = "); Serial.print(Tmp / 340.00 + 36.53);
  
  Serial.print("\nGyX = "); Serial.print(GyX);
  Serial.print(", GyY = "); Serial.print(GyY);
  Serial.print(", GyZ = "); Serial.println(GyZ);

  Serial.println(change);
  Serial.print(r);
  Serial.print(g);
  Serial.println(b);

  float(temp) = Tmp / 340.00 + 36.53;

  if(millis()-lastMs >= 1000)
  {
    char array[40];
    lastMs=millis();
    if(change == 0){
      snprintf(array,sizeof(array),"Temp : %.2f°C",temp);
      myMQTTClient.publish("MJU/IOT/60181793",array);
    }
    else if(change == 1){
      snprintf(array,sizeof(array),"Accle(x,y,z) : %.2f, %.2f, %.2f",valueAcX,valueAcY,valueAcZ);
      myMQTTClient.publish("MJU/IOT/60181793",array);
    }
    
  }
  neopixelWrite(RGB_BUILTIN,r,g,b);
  myMQTTClient.loop();
}
