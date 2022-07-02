/*
   pwm_mapping.cpp - wheel velocity to motor pwm mapping test
   for normal and inverted pwm range
*/

#include <iostream>
#include <math.h> 
using namespace std;
#define MIN_VELOCITY 0.0
#define MAX_VELOCITY 0.728485253

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main()
{   
    cout<<"Assume vel=0.3m/s, MIN_VELOCITY=0, MAX_VELOCITY=0.728485253"<<endl<<endl;
    float mapped_pwm;
    int MIN_PWM, MAX_PWM;
    float vel = 0.3;
    int dir = (vel>=0)?1:-1;
    
    if(fabs(vel)>MAX_VELOCITY){
        vel = dir*MAX_VELOCITY;
    }
    else if(fabs(vel)<MIN_VELOCITY){
        vel = dir*MIN_VELOCITY;
    }
    
    cout<<"Normal pwm range,"<<endl;
    MIN_PWM=26 ; MAX_PWM=229;
    mapped_pwm = mapFloat(fabs(vel), MIN_VELOCITY, MAX_VELOCITY, MIN_PWM, MAX_PWM);
    cout<<"mapped_pwm = "<<mapped_pwm<<endl;
    cout<<"input to motor = 255-mapped_pwm = "<<(255-mapped_pwm)*dir<<endl<<endl;

    cout<<"Inverted pwm range,"<<endl;
    MIN_PWM=229; MAX_PWM=26;
    mapped_pwm = mapFloat(fabs(vel), MIN_VELOCITY, MAX_VELOCITY, MIN_PWM, MAX_PWM);
    cout<<"mapped_pwm = "<<mapped_pwm<<endl;
    cout<<"input to motor = mapped_pwm = "<<mapped_pwm*dir<<endl;
    
    //should get the same input to motor for both normal and inverted pwm

    return 0;
}