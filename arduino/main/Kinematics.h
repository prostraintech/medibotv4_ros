/*
   Kinematics.h 
   - Use instances of Motor.h and LED.h classes
   - Operate left and right motors simultaneously
   - Control left and right LEDs based on the movement, ROS connection, and emergency stop
*/
#pragma once
#include "Config.h"
#include "Motor.h"
#include "LED.h"

class Kinematics{
  public:
    //Kinematics(Motor, Motor); //2WD
    Kinematics(Motor, Motor, LED, LED); //2WD with 2 LEDs
    void Move(int lpwm, int rpwm); //actuate both motors according to pwm/direction
    int isRosConnected;
  private:
    Motor LH_motor;
    Motor RH_motor;
    LED LH_led;
    LED RH_led;
};

// Kinematics::Kinematics(Motor LH_motor, Motor RH_motor){
//   this->LH_motor = LH_motor;
//   this->RH_motor = RH_motor;
// }

Kinematics::Kinematics(Motor LH_motor, Motor RH_motor, LED LH_led, LED RH_led)
: LH_motor(LH_motor), RH_motor(RH_motor), LH_led(LH_led), RH_led(RH_led)
{

}

void Kinematics::Move(int lpwm, int rpwm){
  //add "-" sign to invert direction if needed
  this->LH_motor.Rotate(-lpwm);
  this->RH_motor.Rotate(rpwm);
  
  //move -> blue
  char mv_clr = 'b'; 
  //stop -> white if ros connected, else (yellow if ros not connected else red/estop)
  char stp_clr = this->isRosConnected>0? 'w':(this->isRosConnected==0?'y':'r');

  if(abs(lpwm)<=MIN_PWM && abs(rpwm)<=MIN_PWM){
    this->LH_led.Emit(stp_clr); this->RH_led.Emit(stp_clr);
  }
  else{
      this->LH_led.Emit(mv_clr);  this->RH_led.Emit(mv_clr);
  }
}
