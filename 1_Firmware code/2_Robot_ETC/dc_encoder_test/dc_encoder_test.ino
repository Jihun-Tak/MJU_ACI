// 1440 펄스 / 4 
#define en_chA 2
#define en_chB 3
#define md_in1 5
#define md_in2 6
#define md_in3 10
#define md_in4 11

#define wheel_circum 321.825 //바퀴 둘레 [mm]
#define distance_per_pulse 0.894 //펄스 당 이동거리 [mm]

const byte interruptPin = 2;  // Interrupt pin: D2
unsigned long count = 0;
int prev_rot_count = 0;
int target = 0;
int target_pulse = 0;

void setup() {
  Serial.begin(115200);
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(md_in1, OUTPUT);
  pinMode(md_in2, OUTPUT);
  pinMode(md_in3, OUTPUT);
  pinMode(md_in4, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(interruptPin),intfunc,RISING);
}

void loop() {
  if(Serial.available()){
    target = Serial.parseInt();
    if(target != 0){
      Serial.print("Input data : ");    Serial.println(target);
      target_pulse = target / distance_per_pulse;
      Serial.print("Target rotation count : ");    Serial.println(target_pulse);
      while(1){
        analogWrite(md_in1, 100);
        analogWrite(md_in2, 0);
        if(prev_rot_count != count){
          Serial.print("Rotation count : ");    Serial.println(count);
          prev_rot_count = count;
        }
        if(prev_rot_count == target_pulse) {
          count = 0;
          analogWrite(md_in1, 0);
          analogWrite(md_in2, 0);
          break;
        }
      }
    }
  }
}

void intfunc(){
  count++;
}
