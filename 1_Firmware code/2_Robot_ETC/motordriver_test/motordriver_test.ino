#define motorpin1 5
#define motorpin2 6
#define motorpin3 10
#define motorpin4 11

void setup() {
  //Serial.begin(115200);
  pinMode(motorpin1, OUTPUT);
  pinMode(motorpin2, OUTPUT);
  pinMode(motorpin3, OUTPUT);
  pinMode(motorpin4, OUTPUT);
}

void loop() {
  left();
  delay(1400);
  stopm();
  delay(1000);

  right();
  delay(1400);
  stopm();
  delay(1000);
}

void forward(){
  analogWrite(motorpin1, 100);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, 100);
  analogWrite(motorpin4, 0);
  //Serial.println("Forward");
}

void backward(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, 100);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, 100);
  //Serial.println("Backward");
}

void left(){
  analogWrite(motorpin1, 100);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, 100);
  //Serial.println("left");
}

void right(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, 100);
  analogWrite(motorpin3, 100);
  analogWrite(motorpin4, 0);
  //Serial.println("right");
}

void stopm(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, 0);
  //Serial.println("stop");
}