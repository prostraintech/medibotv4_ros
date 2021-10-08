//------------------------------------------------------------------------------//
bool USING_ROS_PWM = true;  
bool USING_ROS_DIFF = false; 
bool USING_ARDUINO_SERIAL = false;  
//------------------------------------------------------------------------------//
//----------------------------ROS INITIALIZATION--------------------------------//
#define USE_USBCON
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>
ros::NodeHandle nh;
int pwm = 80; 
int pwm_turn = 60;
double lastCmdVelReceived = 0;
double demandx = 0; // in m/s
double demandz = 0; // in rad/s
double Lkp=500, Lki=900, Lkd=0, Linput, Loutput, Lsetpoint;
double Rkp=500, Rki=900, Rkd=0, Rinput, Routput, Rsetpoint;
PID LH_pid(&Linput, &Loutput, &Lsetpoint, Lkp, Lki, Lkd, DIRECT);
PID RH_pid(&Rinput, &Routput, &Rsetpoint, Rkp, Rki, Rkd, DIRECT);
float Ldiff, Lerror, Lprev;
float Rdiff, Rerror, Rprev;

void cmd_vel_callback( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
  lastCmdVelReceived = millis();
}
void pwm_callback( const std_msgs::Int16& pwm_msg){
  pwm = pwm_msg.data;
}
void pwm_turn_callback( const std_msgs::Int16& pwm_turn_msg){
  pwm_turn = pwm_turn_msg.data;
}
void pwm_control_callback( const std_msgs::Bool& pwm_control_msg){
  USING_ROS_PWM = pwm_control_msg.data;
  USING_ROS_DIFF = !pwm_control_msg.data;
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);
ros::Subscriber<std_msgs::Int16> pwm_sub("pwm", pwm_callback);
ros::Subscriber<std_msgs::Int16> pwm_turn_sub("pwm_turn", pwm_turn_callback);
ros::Subscriber<std_msgs::Bool> pwm_control_sub("pwm_control", pwm_control_callback);
std_msgs::Int16 lwheel_msg;
ros::Publisher lwheel_pub("lwheel", &lwheel_msg);
std_msgs::Int16 rwheel_msg;
ros::Publisher rwheel_pub("rwheel", &rwheel_msg);

//----------------------------GENERAL INITIALIZATION--------------------------------//
#include "Config.h"
#include "Motor.h"
#include "LED.h"
unsigned long currentMillis;
unsigned long previousMillis;
Motor LH_motor(LH_D1, LH_D2, LH_D3, LH_ENA, LH_ENB);
Motor RH_motor(RH_D1, RH_D2, RH_D3, RH_ENA, RH_ENB);
Motor PAN_motor(PAN_D1, PAN_D2);
Motor TILT_motor(TILT_D1, TILT_D2);
LED LH_led(LED_R_LH,LED_G_LH,LED_B_LH);
LED RH_led(LED_R_RH,LED_G_RH,LED_B_RH);
void Move(int lpwm, int rpwm);
//------------------------------------------------------------------------------//

void setup(){
  /////////////ARDUINO SERIAL SECTION////////////
  if(USING_ARDUINO_SERIAL){
    Serial.begin(115200);
  }
  ////////////ROS SECTION////////////
  if(USING_ROS_PWM ||USING_ROS_DIFF){
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(pwm_sub);
    nh.subscribe(pwm_turn_sub);
    nh.advertise(lwheel_pub);
    nh.advertise(rwheel_pub);
    LH_pid.SetMode(AUTOMATIC);
    LH_pid.SetOutputLimits(-100,100);
    LH_pid.SetSampleTime(10);
    RH_pid.SetMode(AUTOMATIC);
    RH_pid.SetOutputLimits(-100,100);
    RH_pid.SetSampleTime(10);
  }
}

void loop(){
  if(USING_ROS_PWM || USING_ROS_DIFF) nh.spinOnce();
  currentMillis = millis();
  if(currentMillis - previousMillis >= LOOPTIME){
    previousMillis = currentMillis;
    /////////////ARDUINO SERIAL SECTION////////////
    if(USING_ARDUINO_SERIAL){
      if(Serial.available()>0){ 
        char c = Serial.read();
        if(c=='w'){//forward
          Move(80, 80);
        }
        if(c=='s'){//reverse
          Move(-80, -80);
        }
        else if(c=='a'){//left
          Move(-60, 60);
        }
        else if(c=='d'){//right
          Move(60, -60);
        }
        else{
          Move(0, 0);
        }
      }
    }
    ////////////ROS SECTION////////////////
    if(USING_ROS_PWM){
      if(demandx>0 && abs(demandz)<1){//forward
        Move(pwm, pwm);
      }
      if(demandx<0 && abs(demandz)<1){//reverse
        Move(-pwm, -pwm);
      }
      else if(demandz>0 && abs(demandx)<0.5){//left
        Move(-pwm_turn, pwm_turn);
      }
      else if(demandz<0 && abs(demandx)<0.5){//right
        Move(pwm_turn, -pwm_turn);
      }
      else if(demandx==0 && demandz==0){
        Move(0, 0);
      }
    }

    if(USING_ROS_DIFF){
      //---------------------METHOD 1 (Open-loop)----------------------//
      
      // float rplusl  = (2*demandx)/WHEEL_RADIUS;
      // float rminusl = (demandz*WHEEL_SEPARATION)/WHEEL_RADIUS;
      // float right_omega = (rplusl+rminusl)/2;
      // float left_omega  = rplusl-right_omega;
      // float right_vel = right_omega*WHEEL_RADIUS;
      // float left_vel  = left_omega *WHEEL_RADIUS;
      // float left_pwm = min(((left_vel/MAX_VEL)*MAX_PWM),MAX_PWM);
      // float right_pwm = min(((right_vel/MAX_VEL)*MAX_PWM),MAX_PWM);
      // Move(left_pwm,right_pwm);

      //---------------------METHOD 2 (Open-loop)----------------------//
      
      // float left_vel = (msg.linear.x - msg.angular.z) / 2;
      // float right_vel = (msg.linear.x + msg.angular.z) / 2;
      // float left_pwm = left_vel*(MAX_PWM-MIN_PWM)+MIN_PWM;
      // float right_pwm = right_vel*(MAX_PWM-MIN_PWM)+MIN_PWM;
      // int lsign=(l>0)?1:((l<0)?-1:0);
      // int rsign=(r>0)?1:((r<0)?-1:0);
      // Move(lsign*left_pwm,lsign*right_pwm);

      //---------------------METHOD 3 (Open-loop)----------------------//

      double  left_vel = demandx - (demandz*WHEEL_SEPARATION/2);
      double right_vel = demandx + (demandz*WHEEL_SEPARATION/2);
      double  left_pwm = fabs(left_vel*MAX_PWM/MAX_VEL);
      double right_pwm = fabs(right_vel*MAX_PWM/MAX_VEL);
      left_pwm  = left_pwm<MIN_PWM?MIN_PWM:left_pwm;
      right_pwm = right_pwm<MIN_PWM?MIN_PWM:right_pwm;
      int lsign=(left_vel>0)?1:((left_vel<0)?-1:0);
      int rsign=(right_vel>0)?1:((right_vel<0)?-1:0);
      Move(lsign*left_pwm,lsign*right_pwm);
      
      //---------------------METHOD 4 (Closed-loop)----------------------//

      // double left_vel = demandx - (demandz*WHEEL_SEPARATION/2);
      // double right_vel = demandx + (demandz*WHEEL_SEPARATION/2);
      // Ldiff = LH_motor.getEncoderPos() - Lprev; 
      // Rdiff = RH_motor.getEncoderPos() - Rprev; 
      // Lerror = (demand1*0.95) - Ldiff;
      // Rerror = (demand2*0.95) - Rdiff;
      // Lprev = LH_motor.getEncoderPos(); 
      // Rprev = RH_motor.getEncoderPos();
      // Lsetpoint = left_vel*0.95;
      // Rsetpoint = right_vel*0.95;
      // Linput = Ldiff;
      // Rinput = Rdiff;
      // LH_pid.Compute();
      // RH_pid.Compute();
      // Move(Loutput,Routput);
    }

    if(USING_ROS_PWM || USING_ROS_DIFF){
      lwheel_msg.data = LH_motor.getEncoderPos();
      lwheel_pub.publish(&lwheel_msg);
      rwheel_msg.data = RH_motor.getEncoderPos();
      rwheel_pub.publish(&rwheel_msg);
    }
  }

  if(USING_ROS_PWM || USING_ROS_DIFF){
    //Stop the robot if there are no cmd_vel messages
    if(millis() - lastCmdVelReceived > 500){
      Move(0, 0);
      LH_led.Emit('w');RH_led.Emit('w');
    } 
  }
}


void Move(int lpwm, int rpwm){
  LH_motor.Rotate(lpwm);
  RH_motor.Rotate(rpwm);
  if(lpwm==0 && rpwm==0){
    LH_led.Emit('w');RH_led.Emit('w');
  }
  else{
      LH_led.Emit('b');RH_led.Emit('b');
  }
}
