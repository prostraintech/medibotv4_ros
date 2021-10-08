#include <ros/ros.h>
#include <iostream>
using namespace std;
#define COUNT_PER_METER 95
#define WHEEL_SEPARATION 0.498
#define WHEEL_DIAMETER 0.32
#define WHEEL_RADIUS 0.16
#define MIN_PWM 40
#define MAX_PWM 100
#define MIN_VEL 0
#define MAX_VEL 0.5
double demandx = 0; // in m/s
double demandz = 0; // in rad/s

double VelToPwm(double vel, double in_min, double in_max, double out_min, double out_max){
   return (vel - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(int argc, char** argv) {
  ros::init(argc,argv,"diff_sim");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);

  while(ros::ok()){
      cin >> demandx >> demandz;
      //
      double  left_vel = demandx - (demandz*WHEEL_SEPARATION/2);
      double right_vel = demandx + (demandz*WHEEL_SEPARATION/2);
      double  left_pwm = fabs(left_vel*MAX_PWM/MAX_VEL);
      double right_pwm = fabs(right_vel*MAX_PWM/MAX_VEL);
      left_pwm  = left_pwm<MIN_PWM?MIN_PWM:left_pwm;
      right_pwm = right_pwm<MIN_PWM?MIN_PWM:right_pwm;
      int lsign=(left_vel>0)?1:((left_vel<0)?-1:0);
      int rsign=(right_vel>0)?1:((right_vel<0)?-1:0);

      cout << "l   :" << left_vel << " , r   :" << right_vel << endl;
      cout << "lpwm:" << left_pwm*lsign << " , rpwm:" << right_pwm*lsign << endl;
  }

  return 0;
}

