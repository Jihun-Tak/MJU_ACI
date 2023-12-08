int carsensor;

void setup() {
  Serial.begin(115200);
  pinMode(A2,INPUT);
}

void loop() {
  carsensor = analogRead(A2);
  Serial.println(carsensor);
  delay(200);
}
