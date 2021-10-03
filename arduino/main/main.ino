#define USING_ARDUINO_SERIAL true;  
#define USING_ROS false;  

#include "Motor.h"
#include "LED.h"
#define LOOPTIME 10
//LED pins
const int LED_R_LH = 7, LED_G_LH = 8, LED_B_LH = 9;
const int LED_R_RH = 10, LED_G_RH = 11, LED_B_RH = 12;
LED LH_led(LED_R_LH,LED_G_LH,LED_B_LH);
LED RH_led(LED_R_RH,LED_G_RH,LED_B_RH);
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
Motor LH_motor(LH_D1, LH_D2, LH_D3, LH_ENA, LH_ENB);
Motor RH_motor(RH_D1, RH_D2, RH_D3, RH_ENA, RH_ENB);
Motor PAN_motor(PAN_D1, PAN_D2);
Motor TILT_motor(TILT_D1, TILT_D2);
void Move(int lpwm, int rpwm){
  LH_motor.Rotate(lpwm);
  RH_motor.Rotate(rpwm);
  if(lpwm==0 && rpwm==0){
    LH_led.Emit('w');RH_led.Emit('w');
  }
  else{
    if(lpwm==rpwm){
      LH_led.Emit('b');RH_led.Emit('b');
    }
    else if(lpwm<rpwm){
      LH_led.Emit('b');RH_led.Emit('w');
    }
    else{
      LH_led.Emit('w');RH_led.Emit('b');
    }
  }
}
unsigned long currentMillis;
unsigned long previousMillis;


////////////////////////ROS SECTION///////////////////////////
if(USING_ROS){
  #define USE_USBCON
  #include <ros.h>
  #include <std_msgs/Int16.h>
  #include <geometry_msgs/Twist.h>
  float demandx=0; // in m/s
  float demandz=0; // in rad/s
  int pwm = 80; 
  int pwm_turn = 60;
  double lastCmdVelReceived = 0;
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
  ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);
  ros::Subscriber<std_msgs::Int16> pwm_sub("pwm", pwm_callback);
  ros::Subscriber<std_msgs::Int16> pwm_turn_sub("pwm_turn", pwm_turn_callback);
  std_msgs::Int16 lwheel_msg;
  ros::Publisher lwheel_pub("lwheel", &lwheel_msg);
  std_msgs::Int16 rwheel_msg;
  ros::Publisher rwheel_pub("rwheel", &rwheel_msg);
}



void setup(){
  /////////////ARDUINO SERIAL SECTION////////////
  if(USING_ARDUINO_SERIAL){
    Serial.begin(115200);
  }
  ////////////ROS SECTION////////////
  if(USING_ROS){
    ros::NodeHandle nh;
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(pwm_sub);
    nh.subscribe(pwm_turn_sub);
    nh.advertise(lwheel_pub);
    nh.advertise(rwheel_pub);
  }
}

void loop(){
  if(USING_ROS) nh.spinOnce();
  currentMillis = millis();
  if(currentMillis - previousMillis >= LOOPTIME){
    previousMillis = currentMillis;

    /////////////ARDUINO SERIAL SECTION////////////
    if(USING_ARDUINO_SERIAL){
      if(Serial.available()>0){ 
        char c = Serial.read();
        if(c=='w'){//forward
          Move(pwm, pwm);
        }
        if(c=='s'){//reverse
          Move(-pwm, -pwm);
        }
        else if(c=='a'){//left
          Move(-pwm_turn, pwm_turn);
        }
        else if(c=='d'){//right
          Move(pwm_turn, -pwm_turn);
        }
        else{
          Move(0, 0);
        }
      }
    }
    

    ////////////ROS SECTION////////////////
    if(USING_ROS){
      if(demandx>0 && demandz==0){//forward
        Move(pwm, pwm);
      }
      if(demandx<0 && demandz==0){//reverse
        Move(-pwm, -pwm);
      }
      else if(demandx==0 && demandz>0){//left
        Move(-pwm_turn, pwm_turn);
      }
      else if(demandx==0 && demandz<0){//right
        Move(pwm_turn, -pwm_turn);
      }
      else if(demandx==0 && demandz==0){
        Move(0, 0);
      }
      lwheel_msg.data = LH_motor.getEncoderPos();
      lwheel_pub.publish(&lwheel_msg);
      rwheel_msg.data = RH_motor.getEncoderPos();
      rwheel_pub.publish(&rwheel_msg);
    }

  }

  if(USING_ROS){
    //Stop the robot if there are no cmd_vel messages
    if(millis() - lastCmdVelReceived > 100){
      Move(0, 0);
      LH_led.Emit('w');RH_led.Emit('w');
    } 
  }

}
