#include <iostream>
using namespace std;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main()
{   
    cout<<"Assume vel=0.3m/s, MIN_VELOCITY=0, MAX_VELOCITY=0.728485253"<<endl<<endl;
    const float vel = 0.3, MIN_VELOCITY=0, MAX_VELOCITY=0.728485253;
    float mapped_pwm;
    int MIN_PWM, MAX_PWM;
    
    cout<<"Normal pwm range,"<<endl;
    MIN_PWM=26 ; MAX_PWM=229;
    mapped_pwm = mapFloat(vel, MIN_VELOCITY, MAX_VELOCITY, MIN_PWM, MAX_PWM);
    cout<<"mapped_pwm = "<<mapped_pwm<<endl;
    cout<<"input to motor = 255-mapped_pwm = "<<(255-mapped_pwm)<<endl<<endl;

    cout<<"Inverted pwm range,"<<endl;
    MIN_PWM=229; MAX_PWM=26;
    mapped_pwm = mapFloat(vel, MIN_VELOCITY, MAX_VELOCITY, MIN_PWM, MAX_PWM);
    cout<<"mapped_pwm = "<<mapped_pwm<<endl;
    cout<<"input to motor = mapped_pwm = "<<mapped_pwm<<endl;
    
    //should get the same input to motor for both

    return 0;
}