#pragma once

#include "ATmega2560-HW.h"

//#define _16BIT_PWM_

#ifdef _16BIT_PWM_
  #define MAX_VALUE 0xFFFF
  #define INT_PWM   uint16_t
#else
  #define MAX_VALUE 0xFF
  #define INT_PWM   uint8_t
#endif

class DCMotor {
public:
  DCMotor(int,int,int);
  void CW(INT_PWM);
  void CCW(INT_PWM);
  void Stop();
private:  
  int D1; 
  int D2;
  int D3;
  void initPins();
  INT_PWM protectOutput(INT_PWM);
};


DCMotor::DCMotor(int D1, int D2, int D3) {
  this->D1 = D1;
  this->D2 = D2;
  this->D3 = D3;
  DCMotor::initPins();
}

void DCMotor::initPins() {
  pinMode(this->D1, OUTPUT);
  pinMode(this->D2, OUTPUT);
  pinMode(this->D3, OUTPUT);
  DCMotor::Stop();
}

void DCMotor::Stop() {
  analogWrite(D1,0);
  digitalWrite(D2,LOW);
  // digitalWrite(BRAKE,HIGH);
}

void DCMotor::CW(INT_PWM val) {
  // Motor turns forward or CW
  analogWrite(D1,protectOutput(val));
  digitalWrite(D2,HIGH);
  digitalWrite(D3,LOW);
}

void DCMotor::CCW(INT_PWM val) {
  // Motor turns in the inverse direction or CCW
  analogWrite(D1,protectOutput(val));
  digitalWrite(D2,HIGH);
  digitalWrite(D3,HIGH);
}

INT_PWM DCMotor::protectOutput(INT_PWM val) {

  // For security reasons
  val > MAX_VALUE? val = MAX_VALUE : val;

  return val;
}