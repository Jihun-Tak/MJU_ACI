#include <SoftwareSerial.h>

#define relay1 6
#define relay2 5
#define relay3 4

SoftwareSerial mySerial(13,12);
int carsensor = 0;
void setup() {
  pinMode(relay1,OUTPUT); //릴레이1
  pinMode(relay2,OUTPUT); 
  pinMode(relay3,OUTPUT);
  pinMode(A0,INPUT); //차량출입센서
  pinMode(3,OUTPUT); // 연결신호 보내기
  Serial.begin(115200);
  mySerial.begin(115200);
  digitalWrite(relay1,HIGH);
  digitalWrite(relay2,HIGH);
  digitalWrite(relay3,HIGH);

}
unsigned long long lastMs;
int carflagIN =0;
int carflagOUT =0;
int relayflag = 0;
void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(3,HIGH);
  carsensor = analogRead(A0);
  

    if(millis()-lastMs >= 1000)
  { 
    Serial.println(carsensor);
    lastMs=millis();
    if ( carsensor > 70){
      carflagIN++;
    }
    else if(carsensor < 50){
      carflagOUT = 1;
      carflagIN = 0;
    }

    if((carflagIN > 5)&&(carflagOUT == 1)){
      mySerial.println(10000); // 차량 IN 신호
      carflagOUT = 0;
      Serial.println("IN");
    }
  }
  if(Serial.available()>0){
    int v = Serial.parseInt();
    mySerial.println(v);
  }
  if(mySerial.available()>0){
     relayflag = mySerial.parseInt();
     Serial.println(relayflag);

     if (relayflag == 20000){ //릴레이 ON
        digitalWrite(relay1,LOW);
        digitalWrite(relay2,LOW);
        digitalWrite(relay3,LOW);
     }
     if (relayflag == 30000){ //릴레이 OFF
        digitalWrite(relay1,HIGH);
        digitalWrite(relay2,HIGH);
        digitalWrite(relay3,HIGH);
     }
  }
}
