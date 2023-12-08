#include <SoftwareSerial.h>

SoftwareSerial bluetooth(2,3);

float vout = 0.0;
float vout2 = 0.0;
float vout3 = 0.0;
float vin = 0.0;
float vin2 = 0.0;
float vin3 = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
int A = 0;
float B=0;
float C=0;
float D=0;
float M=0; 
int SPIN = 32;
int SPIN2 = 33;
int SPIN3 = 34;
int value = 0;
int value2 = 0;
int value3 = 0;

void setup(){

pinMode(SPIN, INPUT);
pinMode(SPIN2, INPUT);
pinMode(SPIN3, INPUT);
 

Serial.begin(9600);
//bluetooth.begin(9600);
//Serial.print("Voltage:");

}

 

void loop(){

//if (bluetooth.available())
//{
//  Serial.write(bluetooth.read());
//}
//if (Serial.available())
//{
//  bluetooth.write(Serial.read());
//}

value = analogRead(SPIN);

vout = (value * 5.0) / 1024.0;

vin = vout / ( R2 / ( R1 + R2) );
//Serial.print("V1: ");

//Serial.println(vin,2);

if (vin > 4.23 && vin <= 4.25)
{
  B=100;
 }
else if(vin > 4.21 && vin <= 4.23)
{
  B=98.68;
}else if(vin > 4.19 && vin <= 4.21)
{
  B=96.04;
}else if(vin > 4.17 && vin <= 4.19)
{
  B=94.72;
}else if(vin > 4.15 && vin <= 4.17)
{
  B=93.4;
}else if(vin > 4.13 && vin <= 4.15)
{
  B=92.08;
}else if(vin > 4.11 && vin <= 4.13)
{
  B=90.76;
}else if(vin > 4.09 && vin <= 4.11)
{
  B=89.44;
}else if(vin > 4.08 && vin <= 4.10)
{
  B=88.78;
}else if(vin > 4.06 && vin <= 4.08)
{
  B=87.46;
}else if(vin > 4.04 && vin <= 4.06)
{
  B=86.14;
}else if(vin > 4.02 && vin <= 4.04)
{
  B=84.82;
}else if(vin > 4.00 && vin <= 4.02)
{
  B=83.5;
}else if(vin > 3.98 && vin <= 4.00)
{
  B=82.18;
}else if(vin > 3.96 && vin <= 3.98)
{
  B=80.86;
}else if(vin > 3.94 && vin <= 3.96)
{
  B=79.54;
}else if(vin > 3.92 && vin <= 3.94)
{
  B=78.22;
}else if(vin > 3.90 && vin <= 3.92)
{
  B=76.9;
}else if(vin > 3.88 && vin <= 3.90)
{
  B=75.58;
}else if(vin > 3.86 && vin <= 3.88)
{
  B=74.26;
}else if(vin > 3.84 && vin <= 3.86)
{
  B=72.94;
}else if(vin > 3.82 && vin <= 3.84)
{
  B=71.62;
}else if(vin > 3.80 && vin <= 3.82)
{
  B=70;
}else if(vin > 3.79 && vin <= 3.80)
{
  B=68;
}else if(vin > 3.78 && vin <= 3.79)
{
  B=66;
}else if(vin > 3.77 && vin <= 3.78)
{
  B=64;
}else if(vin > 3.76 && vin <= 3.77)
{
  B=62;
}else if(vin > 3.75 && vin <= 3.76)
{
  B=60;
}else if(vin > 3.74 && vin <= 3.75)
{
  B=58;
}else if(vin > 3.73 && vin <= 3.74)
{
  B=56;
}else if(vin > 3.72 && vin <= 3.73)
{
  B=54;
}else if(vin > 3.71 && vin <= 3.72)
{
  B=52;
}else if(vin > 3.70 && vin <= 3.71)
{
  B=50;
}else if(vin > 3.69 && vin <= 3.70)
{
  B=47;
}else if(vin > 3.68 && vin <= 3.69)
{
  B=46;
}else if(vin > 3.67 && vin <= 3.68)
{
  B=44;
}else if(vin > 3.66 && vin <= 3.67)
{
  B=41;
}else if(vin > 3.65 && vin <= 3.66)
{
  B=38;
}else if(vin > 3.64 && vin <= 3.65)
{
  B=35;
}else if(vin > 3.63 && vin <= 3.64)
{
  B=32;
}else if(vin > 3.62 && vin <= 3.63)
{
  B=29;
}else if(vin > 3.61 && vin <= 3.62)
{
  B=23;
}else if(vin > 3.60 && vin <= 3.61)
{
  B=21;
}else if(vin > 3.59 && vin <= 3.60)
{
  B=20;
}else if(vin > 3.58 && vin <= 3.59)
{
  B=19;
}else if(vin > 3.57 && vin <= 3.58)
{
  B=18;
}else if(vin > 3.56 && vin <= 3.57)
{
  B=17;
}else if(vin > 3.55 && vin <= 3.56)
{
  B=16;
}else if(vin > 3.54 && vin <= 3.55)
{
  B=15;
}else if(vin > 3.53 && vin <= 3.54)
{
  B=14;
}else if(vin > 3.52 && vin <= 3.53)
{
  B=13;
}else if(vin > 3.51 && vin <= 3.52)
{
  B=12;
}else if(vin > 3.50 && vin <= 3.51)
{
  B=11;
}
 //Serial.print("1Cell SOC : ");
 //Serial.print(B);
 //Serial.print("%");
 //Serial.print("\n");




value2 = analogRead(SPIN2);

vout2 = (value2 * 5.0) / 1024.0;

vin2 = vout2 / ( R2 / ( R1 + R2) );

//Serial.print("V2: ");

//Serial.println(vin2,2);


if (vin2 > 4.23 && vin2 <= 4.25)
{
  C=100;
 }
else if(vin2 > 4.21 && vin2 <= 4.23)
{
  C=98.68;
}else if(vin2 > 4.19 && vin2 <= 4.21)
{
  C=96.04;
}else if(vin2 > 4.17 && vin2 <= 4.19)
{
  C=94.72;
}else if(vin2 > 4.15 && vin2 <= 4.17)
{
  C=93.4;
}else if(vin2 > 4.13 && vin2 <= 4.15)
{
  C=92.08;
}else if(vin2 > 4.11 && vin2 <= 4.13)
{
  C=90.76;
}else if(vin2 > 4.09 && vin2 <= 4.11)
{
  C=89.44;
}else if(vin2 > 4.08 && vin2 <= 4.10)
{
  C=88.78;
}else if(vin2 > 4.06 && vin2 <= 4.08)
{
  C=87.46;
}else if(vin2 > 4.04 && vin2 <= 4.06)
{
  C=86.14;
}else if(vin2 > 4.02 && vin2 <= 4.04)
{
  C=84.82;
}else if(vin2 > 4.00 && vin2 <= 4.02)
{
  C=83.5;
}else if(vin2 > 3.98 && vin2 <= 4.00)
{
  C=82.18;
}else if(vin2 > 3.96 && vin2 <= 3.98)
{
  C=80.86;
}else if(vin2 > 3.94 && vin2 <= 3.96)
{
  C=79.54;
}else if(vin2 > 3.92 && vin2 <= 3.94)
{
  C=78.22;
}else if(vin2 > 3.90 && vin2 <= 3.92)
{
  C=76.9;
}else if(vin2 > 3.88 && vin2 <= 3.90)
{
  C=75.58;
}else if(vin2 > 3.86 && vin2 <= 3.88)
{
  C=74.26;
}else if(vin2 > 3.84 && vin2 <= 3.86)
{
  C=72.94;
}else if(vin2 > 3.82 && vin2 <= 3.84)
{
  C=71.62;
}else if(vin2 > 3.80 && vin2 <= 3.82)
{
  C=70;
}else if(vin2 > 3.79 && vin2 <= 3.80)
{
  C=68;
}else if(vin2 > 3.78 && vin2 <= 3.79)
{
  C=66;
}else if(vin2 > 3.77 && vin2 <= 3.78)
{
  C=64;
}else if(vin2 > 3.76 && vin2 <= 3.77)
{
  C=62;
}else if(vin2 > 3.75 && vin2 <= 3.76)
{
  C=60;
}else if(vin2 > 3.74 && vin2 <= 3.75)
{
  C=58;
}else if(vin2 > 3.73 && vin2 <= 3.74)
{
  C=56;
}else if(vin2 > 3.72 && vin2 <= 3.73)
{
  C=54;
}else if(vin2 > 3.71 && vin2 <= 3.72)
{
  C=52;
}else if(vin2 > 3.70 && vin2 <= 3.71)
{
  C=50;
}else if(vin2 > 3.69 && vin2 <= 3.70)
{
  C=47;
}else if(vin2 > 3.68 && vin2 <= 3.69)
{
  C=46;
}else if(vin2 > 3.67 && vin2 <= 3.68)
{
  C=44;
}else if(vin2 > 3.66 && vin2 <= 3.67)
{
  C=41;
}else if(vin2 > 3.65 && vin2 <= 3.66)
{
  C=38;
}else if(vin2 > 3.64 && vin2 <= 3.65)
{
  C=35;
}else if(vin2 > 3.63 && vin2 <= 3.64)
{
  C=32;
}else if(vin2 > 3.62 && vin2 <= 3.63)
{
  C=29;
}else if(vin2 > 3.61 && vin2 <= 3.62)
{
  C=23;
}else if(vin2 > 3.60 && vin2 <= 3.61)
{
  C=21;
}else if(vin2 > 3.59 && vin2 <= 3.60)
{
  C=20;
}else if(vin2 > 3.58 && vin2 <= 3.59)
{
  C=19;
}else if(vin2 > 3.57 && vin2 <= 3.58)
{
  C=18;
}else if(vin2 > 3.56 && vin2 <= 3.57)
{
  C=17;
}else if(vin2 > 3.55 && vin2 <= 3.56)
{
  C=16;
}else if(vin2 > 3.54 && vin2 <= 3.55)
{
  C=15;
}else if(vin2 > 3.53 && vin2 <= 3.54)
{
  C=14;
}else if(vin2 > 3.52 && vin2 <= 3.53)
{
  C=13;
}else if(vin2 > 3.51 && vin2 <= 3.52)
{
  C=12;
}else if(vin2 > 3.50 && vin2 <= 3.51)
{
  C=11;
}
 //Serial.print("2Cell SOC : ");
 //Serial.print(C);
 //Serial.print("%");
 //Serial.print("\n");


value3 = analogRead(SPIN3);

vout3 = (value3 * 5.0) / 1024.0;

vin3 = vout3 / ( R2 / ( R1 + R2) );

//Serial.print("V3: ");
//Serial.println(vin3,2);
if (vin3 > 4.23 && vin3 <= 4.25)
{
  D=100;
 }
else if(vin3 > 4.21 && vin3 <= 4.23)
{
  D=98.68;
}else if(vin3 > 4.19 && vin3 <= 4.21)
{
  D=96.04;
}else if(vin3 > 4.17 && vin3 <= 4.19)
{
  D=94.72;
}else if(vin3 > 4.15 && vin3 <= 4.17)
{
  D=93.4;
}else if(vin3 > 4.13 && vin3 <= 4.15)
{
  D=92.08;
}else if(vin3 > 4.11 && vin3 <= 4.13)
{
  D=90.76;
}else if(vin3 > 4.09 && vin3 <= 4.11)
{
  D=89.44;
}else if(vin3 > 4.08 && vin3 <= 4.10)
{
  D=88.78;
}else if(vin3 > 4.06 && vin3 <= 4.08)
{
  D=87.46;
}else if(vin3 > 4.04 && vin3 <= 4.06)
{
  D=86.14;
}else if(vin3 > 4.02 && vin3 <= 4.04)
{
  D=84.82;
}else if(vin3 > 4.00 && vin3 <= 4.02)
{
  D=83.5;
}else if(vin3 > 3.98 && vin3 <= 4.00)
{
  D=82.18;
}else if(vin3 > 3.96 && vin3 <= 3.98)
{
  D=80.86;
}else if(vin3 > 3.94 && vin3 <= 3.96)
{
  D=79.54;
}else if(vin3 > 3.92 && vin3 <= 3.94)
{
  D=78.22;
}else if(vin3 > 3.90 && vin3 <= 3.92)
{
  D=76.9;
}else if(vin3 > 3.88 && vin3 <= 3.90)
{
  D=75.58;
}else if(vin3 > 3.86 && vin3 <= 3.88)
{
  D=74.26;
}else if(vin3 > 3.84 && vin3 <= 3.86)
{
  D=72.94;
}else if(vin3 > 3.82 && vin3 <= 3.84)
{
  D=71.62;
}else if(vin3 > 3.80 && vin3 <= 3.82)
{
  D=70;
}else if(vin3 > 3.79 && vin3 <= 3.80)
{
  D=68;
}else if(vin3 > 3.78 && vin3 <= 3.79)
{
  D=66;
}else if(vin3 > 3.77 && vin3 <= 3.78)
{
  D=64;
}else if(vin3 > 3.76 && vin3 <= 3.77)
{
  D=62;
}else if(vin3 > 3.75 && vin3 <= 3.76)
{
  D=60;
}else if(vin3 > 3.74 && vin3 <= 3.75)
{
  D=58;
}else if(vin3 > 3.73 && vin3 <= 3.74)
{
  D=56;
}else if(vin3 > 3.72 && vin3 <= 3.73)
{
  D=54;
}else if(vin3 > 3.71 && vin3 <= 3.72)
{
  D=52;
}else if(vin3 > 3.70 && vin3 <= 3.71)
{
  D=50;
}else if(vin3 > 3.69 && vin3 <= 3.70)
{
  D=47;
}else if(vin3 > 3.68 && vin3 <= 3.69)
{
  D=46;
}else if(vin3 > 3.67 && vin3 <= 3.68)
{
  D=44;
}else if(vin3 > 3.66 && vin3 <= 3.67)
{
  D=41;
}else if(vin3 > 3.65 && vin3 <= 3.66)
{
  D=38;
}else if(vin3 > 3.64 && vin3 <= 3.65)
{
  D=35;
}else if(vin3 > 3.63 && vin3 <= 3.64)
{
  D=32;
}else if(vin3 > 3.62 && vin3 <= 3.63)
{
  D=29;
}else if(vin3 > 3.61 && vin3 <= 3.62)
{
  D=23;
}else if(vin3 > 3.60 && vin3 <= 3.61)
{
  D=21;
}else if(vin3 > 3.59 && vin3 <= 3.60)
{
  D=20;
}else if(vin3 > 3.58 && vin3 <= 3.59)
{
  D=19;
}else if(vin3 > 3.57 && vin3 <= 3.58)
{
  D=18;
}else if(vin3 > 3.56 && vin3 <= 3.57)
{
  D=17;
}else if(vin3 > 3.55 && vin3 <= 3.56)
{
  D=16;
}else if(vin3 > 3.54 && vin3 <= 3.55)
{
  D=15;
}else if(vin3 > 3.53 && vin3 <= 3.54)
{
  D=14;
}else if(vin3 > 3.52 && vin3 <= 3.53)
{
  D=13;
}else if(vin3 > 3.51 && vin3 <= 3.52)
{
  D=12;
}else if(vin3 > 3.50 && vin3 <= 3.51)
{
  D=11;
}

M=(B+C+D)/3;
 //Serial.print("3Cell SOC : ");
 //Serial.print(D);
 //Serial.print("%");
 //Serial.print("\n");

Serial.print("Cell SOC : ");
 Serial.print(M);
 Serial.print("%");
 Serial.print("\n");

delay(3000);

}