#include <SoftwareSerial.h>

#define relay_PA_1 2
#define relay_PA_2 3
#define relay_PA_3 4
#define relay_PA_4 5

#define relay_SE_1 6
#define relay_SE_2 7
#define relay_SE_3 8

SoftwareSerial mySerial(13,12);

void setup() {              // 병렬 OFF : LOW
  Serial.begin(115200);     //      ON : HIGH
  mySerial.begin(115200);   // 직렬 OFF : HIGH
  relay_setup();            //      ON : LOW
}

void relay_setup(){
  pinMode(relay_PA_1,OUTPUT);
  digitalWrite(relay_PA_1, LOW);
  pinMode(relay_PA_2,OUTPUT);
  digitalWrite(relay_PA_2, LOW);
  pinMode(relay_PA_3,OUTPUT);
  digitalWrite(relay_PA_3, LOW);
  pinMode(relay_PA_4,OUTPUT);
  digitalWrite(relay_PA_4, LOW);

  delay(300);

  pinMode(relay_SE_1,OUTPUT);
  digitalWrite(relay_SE_1, HIGH);
  pinMode(relay_SE_2,OUTPUT);
  digitalWrite(relay_SE_2, HIGH);
  pinMode(relay_SE_3,OUTPUT);
  digitalWrite(relay_SE_3, HIGH);

  delay(300);
}

void set_relay_Off(){
  digitalWrite(relay_PA_1, LOW);  //병렬 OFF
  digitalWrite(relay_PA_2, LOW);
  digitalWrite(relay_PA_3, LOW);
  digitalWrite(relay_PA_4, LOW);

  delay(1000);

  digitalWrite(relay_SE_1, HIGH); //직렬 OFF
  digitalWrite(relay_SE_2, HIGH);
  digitalWrite(relay_SE_3, HIGH);

  delay(1000);
}

void set_relay_Serial(){
  digitalWrite(relay_PA_1, LOW);  //병렬 OFF
  digitalWrite(relay_PA_2, LOW);
  digitalWrite(relay_PA_3, LOW);
  digitalWrite(relay_PA_4, LOW);

  delay(1000);

  digitalWrite(relay_SE_1, HIGH); //직렬 OFF
  digitalWrite(relay_SE_2, HIGH);
  digitalWrite(relay_SE_3, HIGH);

  delay(1000);

  digitalWrite(relay_PA_1, HIGH);  //병렬 ON
  digitalWrite(relay_PA_2, HIGH);
  digitalWrite(relay_PA_3, HIGH);
  digitalWrite(relay_PA_4, HIGH);

  delay(1000);
}

void set_relay_Parallel(){
  digitalWrite(relay_PA_1, LOW);  //병렬 OFF
  digitalWrite(relay_PA_2, LOW);
  digitalWrite(relay_PA_3, LOW);
  digitalWrite(relay_PA_4, LOW);

  delay(1000);

  digitalWrite(relay_SE_1, HIGH); //직렬 OFF
  digitalWrite(relay_SE_2, HIGH);
  digitalWrite(relay_SE_3, HIGH);

  delay(1000);

  digitalWrite(relay_SE_1, LOW);  //직렬 ON
  digitalWrite(relay_SE_2, LOW);
  digitalWrite(relay_SE_3, LOW);

  delay(1000);
}

void loop() {
  if(mySerial.available()>0){
    int data = mySerial.parseInt();
    if(data == 1) set_relay_Off();
    else if(data == 2) set_relay_Serial();
    else if(data == 3) set_relay_Parallel();
  }
}
