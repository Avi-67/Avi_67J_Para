#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const unsigned int ADDRESS = 0x27;
const int CHARS_NUM = 16;
const int LINES_NUM = 2;

const int Switch_PIN = 7;
const int SERVO_PIN = 4;
int btnPushCnt = 0, old_btnPushCnt = 0;

Servo myServo;

LiquidCrystal_I2C lcd(ADDRESS, CHARS_NUM, LINES_NUM);

void IRAM_ATTR SwitchPushed() {
  btnPushCnt++;
}

void openthedoor(){
  delay(1000);
  myServo.write(90);
}

void closethedoor(){
  delay(1000);
  myServo.write(0);
}

void setup() {
  delay(1000);
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);
  pinMode(32,OUTPUT);
  Serial.print("Hello,world!");
  pinMode(Switch_PIN, INPUT_PULLUP);  // プルアップ設定
  attachInterrupt(Switch_PIN, SwitchPushed, FALLING);  // 割り込み処理設定
}

void loop() {
  if(old_btnPushCnt != btnPushCnt){
    if(btnPushCnt % 2 == 1){
      openthedoor();
      Serial.print("Hello,world!");
      old_btnPushCnt = btnPushCnt;
    }else{
      closethedoor();
      Serial.print("Hello,world!");
      old_btnPushCnt = btnPushCnt;
    }
  }
  //Serial.println("Hello,world!");
  // put your main code here, to run repeatedly:
}

// void loop() {
//   myServo.write(0);
//   delay(1000);
//   myServo.write(90);
//   delay(1000);
//   Serial.print("Hello,world!");
  
// }