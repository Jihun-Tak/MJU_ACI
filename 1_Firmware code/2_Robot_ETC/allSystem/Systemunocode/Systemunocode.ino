#include <SoftwareSerial.h>

#define STEP_DELAY 700
#define STEP_PIN 8
#define DIR_PIN 7
#define ENA_PIN 4
#define motorpin1 5
#define motorpin2 6
#define motorpin3 10
#define motorpin4 11
#define Mag_PIN 9


SoftwareSerial mySerial(13,12);

int motor_speed = 100;
float wheel_dia = 322; //102.44;
float distance_per_pulse = wheel_dia / 360;
float turn_factor = ((270*PI)/4)/distance_per_pulse;
int checkflag = 0;
int LRflag = 0;

const byte interruptPin1 = 2; //left
const byte interruptPin2 = 3; //right
unsigned long count1 = 0;
unsigned long count2 = 0;
int prev_rot_count1 = 0;
int prev_rot_count2 = 0;
int dc_target = 0;
int target_pulse = 0;
int target_pulse1 = 0;
int target_pulse2 = 0;
int left_flag = 0;
int right_flag = 0;
int start_flag = 0;

int checkpoint;



int target_pix = 0;
int target_mm = 0;
int target_temp = 0;
int target_value = 0;
int target_delay = 0;
int target_check1 = 0;
int target_check2 = 0;
int current_value = 0;
int target_middle = map(240, 480, 0, 50, 300);
int target_a = 0;
int temp = 0;

int flagy = 1;
int flagz = 0;

unsigned long long lastMs;
unsigned long long backDCcheck; // back움직임 값
int backDCsave = 0;
int backDCflag = 0; //back움직임 flag

void setup()
{
  Serial.begin(115200);
  mySerial.begin(115200);
  Serial.println("Serial connected!");
  motor_setup();
  attachInterrupt(digitalPinToInterrupt(interruptPin1),intfunc1,RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2),intfunc2,RISING);
  delay(100);
  //target_mm = 200;
  //target_temp = target_mm;
  //target_value = (target_temp - 50) * 20;
  //Serial.print("target to : "); Serial.println(target_value);
  //delay(10);
  //step_move();

  while(1){

  if(mySerial.available()>0){   
    target_pix = mySerial.parseInt();
    Serial.print("Data income : "); Serial.println(target_pix);
    if(target_pix != 0){
      if((target_pix>=30000)&&(target_pix < 40000)){ //STEP 입력값은 30000 ~ 39999 / 성공시 9999
        target_check1 = target_pix - 30000;
        target_pix = 240 - target_pix;
        target_mm = target_pix * 0.33;
        Serial.print("check : "); Serial.print(target_check1);
        Serial.print("  |   pixel : "); Serial.println(target_pix);
        if(target_mm < 500 && target_mm > -500){
          Serial.print("offset : "); Serial.print(target_mm);

          target_temp += target_mm;

          Serial.print("  |   current : "); Serial.print(target_temp);
          target_value = (target_temp - 50) * 20;

          Serial.print("  |   target_v : "); Serial.println(target_value);
          if(target_value <= 0) target_value = 0;
          else if(target_value >= 5340) target_value = 5340;
          step_move();
        }
        else if (target_check1 == 9999){
          Serial.println("Z axis success!!");
          target_temp += 53;
          Serial.print("current : "); Serial.println(target_temp);
          //target_value = (target_temp - 50) * 20; // 얘가 찾는코드
          target_value = 4000; //얘가 스텝 고정코드
          step_move();
          flagz = 0;
        }
      }
      else if(target_pix >= 10000 && target_pix < 11000){
        motor_speed = 100;
        target_mm = target_pix - 10000;
        target_value = (target_mm - 50) * 20;
        step_move();
      }
      
      else if(target_pix == 13000){
        motor_speed = 100;
      }
      else if(target_pix == 15000){
        digitalWrite(Mag_PIN, HIGH);
      }
      else if(target_pix == 16000){
        digitalWrite(Mag_PIN, LOW);
      }
      // DC 영상처리 이동
      else if(target_pix == 19100) //후진
      {
        dc_backward();
        delay(150);
        dc_stop();
      }
      else if(target_pix == 19200){
        dc_backward();
        delay(80);
        dc_stop();
      }
      else if(target_pix == 19300){
        dc_backward();
        delay(20);
        dc_stop();
      }

      else if(target_pix == 19600){ // 전진
        dc_forward();
        delay(150);
        dc_stop();
      }
      else if(target_pix == 19700){
        dc_forward();
        delay(80);
        dc_stop();
      }
      else if(target_pix == 19800){
        dc_forward();
        delay(20);
        dc_stop();
      }


      else if(target_pix == 17700){ // back check
        backDCflag = 1;
      }
      else if(target_pix == 17800){ // 회수 DC
        Serial.print("회수한다 : ");
        Serial.println(backDCsave);
        dc_forward();
        delay(backDCsave + 50);
        dc_stop();
        backDCsave = 0;
      }

      else if(target_pix >= 20000){ //DC 입력값은 전진 20000 ~ 22999 / 후진 23000 ~ 24999 / 좌회전 27777 / 우회전 28888
        dc_target = target_pix - 20000; // 전진 0 ~ 2999 / 후진 3000 ~ 4999 / 좌회전 7777 / 우회전 8888
        if (dc_target != 0){
          if(dc_target > 0 && dc_target <= 2999) start_flag = 0;
          else if((dc_target > 3000)&&(dc_target <= 5999)){
            dc_target -= 3000; //0 ~ 2999로 범위 변경
            start_flag = -1;
          }
          Serial.print("Input data : "); Serial.println(dc_target);
          target_pulse1 = (dc_target / distance_per_pulse) - 10;
          target_pulse2 = (dc_target / distance_per_pulse) - 10;
          Serial.print("Target rotation count : "); Serial.println(target_pulse1+10);
          if(dc_target == 7777) { //left
            start_flag = 2;
            target_pulse1 = turn_factor;
            target_pulse2 = turn_factor;
          }
          if(dc_target == 8888) { //right
            start_flag = 3;
            target_pulse1 = turn_factor;
            target_pulse2 = turn_factor;//(turn_factor / distance_per_pulse) - 10;
          } 
          while(1){
            if(start_flag == 0) {
              dc_forward();
              start_flag = 1;
              LRflag = 0;
            }
            else if(start_flag == -1){
              dc_backward();
              start_flag = 1;
              LRflag = 0;
            }
            else if(start_flag == 2){
              dc_left();
              start_flag = 1;
              LRflag = 1;
            }
            else if(start_flag == 3){
              dc_right();
              start_flag = 1;
              LRflag = 1;
            }

            if(prev_rot_count1 != count1) prev_rot_count1 = count1;
            if(prev_rot_count2 != count2) prev_rot_count2 = count2;

            /*if(left_flag == 0 || right_flag == 0){ //엔코더 카운트
              Serial.print("Left count : "); Serial.print(prev_rot_count1);
              Serial.print("  Right count : "); Serial.println(prev_rot_count2);
            }*/

            if(prev_rot_count1 >= target_pulse1){
              left_stop();
              left_flag = 1;
              checkflag = 1;
            }
            if(prev_rot_count2 >= target_pulse2){
              right_stop();
              right_flag = 1;
              checkflag = 1;
            }
            
            checkpoint = analogRead(A2);
            Serial.print("수발광센싱 : ");
            Serial.println(checkpoint);
            
          
            if((checkpoint < 1005)&&(checkflag == 0)&&(LRflag == 0)){ // 체크포인트 흰색 나오면 멈춤
              Serial.println("체크포인트 도달");
              
              dc_stop();
              left_flag = 1;
              right_flag = 1;
              checkflag = 1;
              mySerial.println(checkflag);
              Serial.println("Checkpoint");
            }

            if((checkflag == 1)&&(checkpoint >= 1020)){
              delay(800);
              checkflag = 0;
            }
            else if(checkpoint >= 1020){
              checkflag = 0;
            }
        
            if(left_flag == 1 && right_flag == 1){
              if(LRflag == 1){
                mySerial.println(1);
                checkflag = 1;
                Serial.println("chackend");
              }
              resetencode();
              //mySerial.println("done!");
              break;
            }
          }
        }
      }
    }
  }
  }
}

  /*
        target_check2 = target_pix;
        target_pix -= 20000;
        target_pix  = 354 - target_pix;
        target_mm   = target_pix * 0.4347826087;
        target_delay = abs(target_mm)* 10;
        Serial.print("check : "); Serial.print(target_check2);
        Serial.print("  |   pixel : "); Serial.print(target_pix);
        Serial.print("  |   goal  : "); Serial.print(target_mm);
        Serial.print("  |   delay : "); Serial.println(target_delay);

        if(target_mm < 0){
          dc_forward();
          delay(50);
          dc_stop();
        }
        else if(target_mm > 0 && target_mm < 90000){
          dc_backward();
          delay(50);
          dc_stop();
        }
        else if(target_check2 == 99999){
          dc_stop();
          flagy = 0;
          flagz = 1;
        }
  */

void loop(){
}

void resetencode(){
  Serial.println("Sensor Reset!");
  count1 = 0;
  count2 = 0;
  left_flag = 0;
  right_flag = 0;
  start_flag = 0;
  prev_rot_count1 = 0;
  prev_rot_count2 = 0;
}

void intfunc1(){
  count1++;
}

void intfunc2(){
  count2++;
}

void motor_setup(){
  pinMode(interruptPin1, INPUT_PULLUP);
  pinMode(interruptPin2, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(ENA_PIN, HIGH);
  pinMode(motorpin1, OUTPUT);
  pinMode(motorpin2, OUTPUT);
  pinMode(motorpin3, OUTPUT);
  pinMode(motorpin4, OUTPUT);
  pinMode(A2,INPUT);
  pinMode(9,OUTPUT);
}

void step_move(){
  while ( target_value > current_value ){ 
    step_forward();  
  }
  while ( target_value < current_value ){ 
    step_backward(); 
  }
  digitalWrite(ENA_PIN, HIGH); 
}

void step_forward()
{
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY);
  current_value++;
}

void step_backward()
{
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY);
  current_value--;
}

void left_stop(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, 0);
  Serial.println("Left motor stop");
}

void right_stop(){
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, 0);
  Serial.println("Right motor stop");
}

void dc_forward(){
  analogWrite(motorpin1, motor_speed);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, motor_speed-10);
  analogWrite(motorpin4, 0);
  Serial.println("Forward");
}

void dc_backward(){
  if(backDCflag == 1){
    lastMs = millis();
  }
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, motor_speed);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, motor_speed-10);
  Serial.println("Backward");
}

void dc_right(){
  analogWrite(motorpin1, motor_speed);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, motor_speed-10);
  Serial.println("right");
}

void dc_left(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, motor_speed);
  analogWrite(motorpin3, motor_speed-10);
  analogWrite(motorpin4, 0);
  Serial.println("left");
}


void dc_stop(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, 0);
  if(backDCflag == 1){
    backDCcheck = millis();
    backDCsave = backDCcheck - lastMs;
    Serial.print("SAVE BACK DATA ---------- ");
    Serial.println(backDCsave);
    backDCflag = 0;
  }
  Serial.println("stop");
}