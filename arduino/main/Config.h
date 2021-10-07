#pragma once
//driving motor and encoder pins
#define LH_D1 3          
#define LH_D2 28      
#define LH_D3 27   
#define LH_ENA 46 
#define LH_ENB 45     
#define RH_D1 4          
#define RH_D2 26         
#define RH_D3 25         
#define RH_ENA 44 
#define RH_ENB 43 
#define BRAKE 29   
//pan tilt motor pins
#define PAN_D1 24
#define PAN_D2 23
#define TILT_D1 22
#define TILT_D2 2
//LED pins
#define LED_R_LH 7 
#define LED_G_LH 8
#define LED_B_LH 9
#define LED_R_RH 10 
#define LED_G_RH 11
#define LED_B_RH 12

//General Constants
#define LOOPTIME 10
//ROS Constants
#define COUNT_PER_METER 95
#define WHEEL_SEPARATION 0.498
#define WHEEL_DIAMETER 0.32
#define WHEEL_RADIUS 0.16
#define MIN_PWM 0
#define MAX_PWM 100
#define MAX_VEL 0.25
