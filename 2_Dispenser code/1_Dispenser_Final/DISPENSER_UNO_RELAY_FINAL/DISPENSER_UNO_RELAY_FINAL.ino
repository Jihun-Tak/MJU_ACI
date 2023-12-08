#define relay_all_1 2  //SMPS
#define relay_all_2 3
#define relay_all_3 4

void setup() {
  Serial.begin(115200);
  pinMode(relay_all_1,OUTPUT);
  pinMode(relay_all_2,OUTPUT);
  pinMode(relay_all_3,OUTPUT);

  digitalWrite(relay_all_1,HIGH);
  digitalWrite(relay_all_2,HIGH);
  digitalWrite(relay_all_3,HIGH);
}

void loop() {
  if(Serial.available()>0){
    int a = Serial.parseInt();
    if(a == 1){
      digitalWrite(relay_all_1,HIGH);
      digitalWrite(relay_all_2,HIGH);
      digitalWrite(relay_all_3,HIGH);
      Serial.println("SMPS 꺼짐");
    }
    else if ( a == 2){
      digitalWrite(relay_all_1,LOW);
      digitalWrite(relay_all_2,LOW);
      digitalWrite(relay_all_3,LOW);
      Serial.println("SMPS 켜짐");      
    }
  }

}
