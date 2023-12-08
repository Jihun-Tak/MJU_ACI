#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

SoftwareSerial mySerial(13,12);

#define SPIN A0
#define SPIN2 A1

unsigned long long lastMS = 0;
float vout = 0.0;
float vout2 = 0.0;
float vin1 = 0.0;
float vin2 = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
int value  = 0;
int value2 = 0;

void setup() {
  pinMode(SPIN, INPUT);
  pinMode(SPIN2, INPUT);
  Serial.begin(115200);
  mySerial.begin(115200);
  Serial.println("Serial connected!");
}

void loop() {
  if(millis() - lastMS >= 3000){
    lastMS = millis();
    value = analogRead(SPIN);
    value2 = analogRead(SPIN2);
    vout = (value * 5.0) / 1024.0;
    vout2 = (value2 * 5.0) / 1024.0;
  
    vin1 = vout / ( R2 / ( R1 + R2) );
    vin2 = vout2 / ( R2 / ( R1 + R2) );
  
    float Bat1 = get_bat_percent(vin1);
    float Bat2 = get_bat_percent(vin2);

    if(Bat1 != 999 && Bat2 != 999){
      int Bat = (Bat1+Bat2)/2;
      mySerial.println(Bat);
      Serial.print("Bat : "); Serial.println(Bat);  
    } 
  }
}

float get_bat_percent(float k){
  float vin = k;
  float B = 0;
  if     (vin > 4.23 && vin <= 4.25) B=100;
  else if(vin > 4.21 && vin <= 4.23) B=98.68; 
  else if(vin > 4.19 && vin <= 4.21) B=96.04;
  else if(vin > 4.17 && vin <= 4.19) B=94.72;
  else if(vin > 4.15 && vin <= 4.17) B=93.4;
  else if(vin > 4.13 && vin <= 4.15) B=92.08;
  else if(vin > 4.11 && vin <= 4.13) B=90.76;
  else if(vin > 4.09 && vin <= 4.11) B=89.44;
  else if(vin > 4.08 && vin <= 4.10) B=88.78;
  else if(vin > 4.06 && vin <= 4.08) B=87.46;
  else if(vin > 4.04 && vin <= 4.06) B=86.14;
  else if(vin > 4.02 && vin <= 4.04) B=84.82;
  else if(vin > 4.00 && vin <= 4.02) B=83.5;
  else if(vin > 3.98 && vin <= 4.00) B=82.18;
  else if(vin > 3.96 && vin <= 3.98) B=80.86;
  else if(vin > 3.94 && vin <= 3.96) B=79.54;
  else if(vin > 3.92 && vin <= 3.94) B=78.22;
  else if(vin > 3.90 && vin <= 3.92) B=76.9;
  else if(vin > 3.88 && vin <= 3.90) B=75.58;
  else if(vin > 3.86 && vin <= 3.88) B=74.26;
  else if(vin > 3.84 && vin <= 3.86) B=72.94;
  else if(vin > 3.82 && vin <= 3.84) B=71.62;
  else if(vin > 3.80 && vin <= 3.82) B=70;
  else if(vin > 3.79 && vin <= 3.80) B=68;
  else if(vin > 3.78 && vin <= 3.79) B=66;
  else if(vin > 3.77 && vin <= 3.78) B=64;
  else if(vin > 3.76 && vin <= 3.77) B=62;
  else if(vin > 3.75 && vin <= 3.76) B=60;
  else if(vin > 3.74 && vin <= 3.75) B=58;
  else if(vin > 3.73 && vin <= 3.74) B=56;
  else if(vin > 3.72 && vin <= 3.73) B=54;
  else if(vin > 3.71 && vin <= 3.72) B=52;
  else if(vin > 3.70 && vin <= 3.71) B=50;
  else if(vin > 3.69 && vin <= 3.70) B=47;
  else if(vin > 3.68 && vin <= 3.69) B=46;
  else if(vin > 3.67 && vin <= 3.68) B=44;
  else if(vin > 3.66 && vin <= 3.67) B=41;
  else if(vin > 3.65 && vin <= 3.66) B=38;
  else if(vin > 3.64 && vin <= 3.65) B=35;
  else if(vin > 3.63 && vin <= 3.64) B=32;
  else if(vin > 3.62 && vin <= 3.63) B=29;
  else if(vin > 3.61 && vin <= 3.62) B=23;
  else if(vin > 3.60 && vin <= 3.61) B=21;
  else if(vin > 3.59 && vin <= 3.60) B=20;
  else if(vin > 3.58 && vin <= 3.59) B=19;
  else if(vin > 3.57 && vin <= 3.58) B=18;
  else if(vin > 3.56 && vin <= 3.57) B=17;
  else if(vin > 3.55 && vin <= 3.56) B=16;
  else if(vin > 3.54 && vin <= 3.55) B=15;
  else if(vin > 3.53 && vin <= 3.54) B=14;
  else if(vin > 3.52 && vin <= 3.53) B=13;
  else if(vin > 3.51 && vin <= 3.52) B=12;
  else if(vin > 3.50 && vin <= 3.51) B=11;
  else if(vin > 3.50 && vin <= 3.51) B=11;
  else if(vin > 3.50 && vin <= 3.51) B=10;
  else if(vin > 3.0 && vin <= 3.5)   B=9;
  else if(vin > 2.8 && vin <= 3.0)   B=8;
  else if(vin > 2.7 && vin <= 2.8)   B=7;
  else if(vin > 2.6 && vin <= 2.7)   B=6;
  else if(vin > 2.5 && vin <= 2.6)   B=5.5;
  else if(vin > 2.4 && vin <= 2.5)   B=5;
  else if(vin > 2.3 && vin <= 2.4)   B=4;
  else if(vin > 2.2 && vin <= 2.3)   B=3;
  else if(vin > 2.0 && vin <= 2.2)   B=2;
  else                               B=999;
  return B;
}