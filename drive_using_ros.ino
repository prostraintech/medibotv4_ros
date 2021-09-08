//wheel radius, r = 160mm = 0.16m
//wheel separation, l = 498mm = 0.498m
//lidar height from ground = 229.4mm
//diff drive parameters in metre
#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>
ros::NodeHandle  nh;
#define WHEEL_SEPARATION 0.498
#define WHEEL_RADIUS 0.16
#define ENCODER_CPM 97
#define LOOPTIME 10
//motor pins
#define LH_D1 3          // Left hand PWM
#define LH_D2 28         // Left hand STOP
#define LH_D3 27         // Left hand DIRECTION
#define LH_A1 A4         // Left hand ANALOG
#define RH_D1 4          // Right hand PWM
#define RH_D2 26         // Right hand STOP
#define RH_D3 25         // Right hand DIRECTION
#define RH_A1 A5         // Right hand ANALOG
#define BR 29         // Brake
float demandx=0; // in m/s
float demandz=0; // in rad/s
unsigned long currentMillis;
unsigned long previousMillis;      

void cmd_vel_callback( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_callback );

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  pinMode(RH_D1,OUTPUT); // motor PWM pins
  pinMode(RH_D2,OUTPUT);
  pinMode(RH_D3,OUTPUT);
  pinMode(LH_D1,OUTPUT);
  pinMode(LH_D2,OUTPUT);
  pinMode(LH_D3,OUTPUT);
  pinMode(BR,OUTPUT);
}

void loop() {
  nh.spinOnce();
  currentMillis = millis();
  if(currentMillis - previousMillis >= LOOPTIME){
    previousMillis = currentMillis;
    int pwm = 70;
    if(demandx<0 && demandz==0){//reverse
      analogWrite(LH_D1,pwm);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,HIGH);
      analogWrite(RH_D1,pwm);
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,HIGH);
    }
    else if(demandx>0 && demandz==0){//forward
      analogWrite(LH_D1,pwm);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,LOW);
      analogWrite(RH_D1,pwm);
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,LOW);
    }
    else if(demandx==0 && demandz>0){//left
      analogWrite(LH_D1,pwm);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,HIGH);
      analogWrite(RH_D1,pwm);
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,LOW);
    }
    else if(demandx==0 && demandz<0){//right
      analogWrite(LH_D1,pwm);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,LOW);
      analogWrite(RH_D1,pwm);
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,HIGH);
    }
    else if(demandx==0 && demandz==0){
      analogWrite(LH_D1,0);//left wheel stop
      digitalWrite(LH_D2,LOW);
      analogWrite(RH_D1,0);//right wheel stop
      digitalWrite(RH_D2,LOW);
      digitalWrite(BR,HIGH);
    }
  }
}
