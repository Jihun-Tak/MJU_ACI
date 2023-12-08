#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup(){
  Serial.begin(115200);
  Serial.print("Serial connected\n");
  lcd.init(); 
  Serial.print("LCD connected\n");
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Driving Mode");
}

void loop(){
  float a = analogRead(A0);
  a=a*5.0/1024.0*15;
  Serial.println(a);
  lcd.setCursor(0,1);
  lcd.print("          ");
  lcd.setCursor(0,1);
  lcd.print(a);

  delay(3000);
}