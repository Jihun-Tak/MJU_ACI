int value;
 
void setup() 
{
  Serial.begin(115200); 
  pinMode(A2, INPUT); 
}
 
void loop() 
{
  value = analogRead(A2); 
  
  Serial.println(value); 
  
  delay(10);
}