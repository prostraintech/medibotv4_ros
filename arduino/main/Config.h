#pragma once
//DRIVING MOTOR PINS
#define LH_D1 5
#define LH_D2 26
#define LH_D3 25
#define LH_ENA 44
#define LH_ENB 43
#define RH_D1 4
#define RH_D2 28
#define RH_D3 27      
#define RH_ENA 46
#define RH_ENB 45
#define BRAKE 29
//PAN & TILT MOTOR PINS
#define PAN_D1 24
#define PAN_D2 23
#define PAN_EN A4
#define TILT_D1 2
#define TILT_D2 3
#define TILT_EN A5
#define PAN_LEFT_LIM 350
#define PAN_RIGHT_LIM 550
#define TILT_UP_LIM 455
#define TILT_DOWN_LIM 550
//LED PINS
#define LED_R_RH 7
#define LED_G_RH 8
#define LED_B_RH 9
#define LED_R_LH 10
#define LED_G_LH 11
#define LED_B_LH 12
//SENSOR PINS
#define IR1 A0
#define IR2 A1
#define IR3 A2
#define SON1 A3
#define CSENS A6
#define LSR1 42
#define LSR2 41
#define LSR3 40
//SWITCH PINS
#define SW_MODE 38
#define ESTOP 39
//PENDANT PINS
#define CS_STT 66
#define CS_STP 67
#define CS_FWD 68
#define CS_RVR 69
#define CS_LFT 48
#define CS_RGT 49
//GENERAL CONSTANT
#define LOOPTIME 10
#define MOTOR_SPEED 30
#define DIFF_MOTOR_SPEED 55
//ROS CONSTANT
#define CMD_VEL_TIMEOUT 300
#define TICKS_PER_METER 95
#define WHEEL_SEPARATION 0.498
#define WHEEL_DIAMETER 0.32
#define WHEEL_RADIUS 0.16
#define MIN_PWM 55
#define MAX_PWM 100
#define MIN_VEL 0
#define MAX_VEL 0.5
#define LH_KP 1
#define LH_KI 0
#define LH_KD 0
#define RH_KP 1
#define RH_KI 0
#define RH_KD 0
