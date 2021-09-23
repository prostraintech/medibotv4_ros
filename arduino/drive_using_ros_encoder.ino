//wheel radius, r = 160mm = 0.16m
//wheel separation, l = 498mm = 0.498m
//lidar height from ground = 229.4mm
//diff drive parameters in metre
#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int16.h>
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
// wheel encoder pins
#define LH_ENA 46 // left encoder A
#define LH_ENB 45 // left encoder B         
#define RH_ENA 44 // right encoder A
#define RH_ENB 43 // right encoder B 
float demandx=0; // in m/s
float demandz=0; // in rad/s
volatile long encoderLH_Pos = 0; // encoder left pos
volatile long encoderRH_Pos = 0; // encoder right pos
unsigned long currentMillis;
unsigned long previousMillis;  
double lastCmdVelReceived = 0;

void cmd_vel_callback( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
  lastCmdVelReceived = millis();
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_callback );
std_msgs::Int16 lwheel_msg;
ros::Publisher lwheel_pub("lwheel", &lwheel_msg);
std_msgs::Int16 rwheel_msg;
ros::Publisher rwheel_pub("rwheel", &rwheel_msg); 

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(lwheel_pub);
  nh.advertise(rwheel_pub);
  pinMode(RH_D1,OUTPUT); // motor PWM pins
  pinMode(RH_D2,OUTPUT);
  pinMode(RH_D3,OUTPUT);
  pinMode(LH_D1,OUTPUT);
  pinMode(LH_D2,OUTPUT);
  pinMode(LH_D3,OUTPUT);
  pinMode(BR,OUTPUT);
  pinMode(LH_ENA, INPUT_PULLUP); // encoder pins
  pinMode(LH_ENB, INPUT_PULLUP);
  pinMode(RH_ENA, INPUT_PULLUP);
  pinMode(RH_ENB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LH_ENA), doEncoderLHA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LH_ENB), doEncoderLHB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENA), doEncoderRHA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENB), doEncoderRHB, CHANGE);
}

void loop() {
  nh.spinOnce();
  currentMillis = millis();
  if(currentMillis - previousMillis >= LOOPTIME){
    previousMillis = currentMillis;
    int pwm = 70;
    int pwm_turn = 70;
    if(demandx<0 && abs(demandz)<=1){//reverse
      analogWrite(LH_D1,pwm);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,HIGH);
      analogWrite(RH_D1,pwm);
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,HIGH);
    }
    else if(demandx>0 && abs(demandz)<=1){//forward
      analogWrite(LH_D1,pwm);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,LOW);
      analogWrite(RH_D1,pwm);
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,LOW);
    }
    else if(abs(demandx)<=0.1 && demandz>0){//left
      analogWrite(LH_D1,pwm_turn);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,HIGH);
      analogWrite(RH_D1,pwm_turn);
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,LOW);
    }
    else if(abs(demandx)<=0.1 && demandz<0){//right
      analogWrite(LH_D1,pwm_turn);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,LOW);
      analogWrite(RH_D1,pwm_turn);
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
    lwheel_pub.publish(&lwheel_msg);
    rwheel_pub.publish(&rwheel_msg);
  }

  //Stop the robot if there are no cmd_vel messages
  if(millis() - lastCmdVelReceived > 100) {
      analogWrite(LH_D1,0);//left wheel stop
      digitalWrite(LH_D2,LOW);
      analogWrite(RH_D1,0);//right wheel stop
      digitalWrite(RH_D2,LOW);
      digitalWrite(BR,HIGH);
  }
}


//*************************encoders interrupt functions******************
//****************** encoder left ********************
void doEncoderLHA(){
  //look for low-to-high on channel A
  if(digitalRead(LH_ENA)==HIGH){
    //check channel B to see which way encoder is turning
    if(digitalRead(LH_ENB)==LOW){
      encoderLH_Pos++; // CW
    }
    else{
      encoderLH_Pos--; // CCW
    }
  }
  else{ //must be a high-to-low edge on channel A
    //check channel B to see which way encoder is turning
    if(digitalRead(LH_ENB)==HIGH){
      encoderLH_Pos++; // CW
    }
    else{
      encoderLH_Pos--; // CCW
    }
  }
  lwheel_msg.data = encoderLH_Pos;
}

void doEncoderLHB(){
  //look for low-to-high on channel B
  if(digitalRead(LH_ENB)==HIGH){
    //check channel A to see which way encoder is turning
    if(digitalRead(LH_ENA)==HIGH){
      encoderLH_Pos++; // CW
    }
    else{
      encoderLH_Pos--; // CCW
    }
  }
  else{ //must be a high-to-low edge on channel B
    //check channel A to see which way encoder is turning
    if(digitalRead(LH_ENA)==LOW){
      encoderLH_Pos++; // CW
    }
    else{
      encoderLH_Pos--; // CCW
    }
  }
  lwheel_msg.data = encoderLH_Pos;
}

//****************** encoder right ********************
void doEncoderRHA(){
  //look for low-to-high on channel A
  if(digitalRead(RH_ENA)==HIGH){
    //check channel B to see which way encoder is turning
    if(digitalRead(RH_ENB)==LOW){
      encoderRH_Pos++; // CW
    }
    else{
      encoderRH_Pos--; // CCW
    }
  }
  else{ //must be a high-to-low edge on channel A
    //check channel B to see which way encoder is turning
    if(digitalRead(RH_ENB)==HIGH){
      encoderRH_Pos++; // CW
    }
    else{
      encoderRH_Pos--; // CCW
    }
  }
  rwheel_msg.data = encoderRH_Pos;
}

void doEncoderRHB(){
  //look for low-to-high on channel B
  if(digitalRead(RH_ENB)==HIGH){
    //check channel A to see which way encoder is turning
    if(digitalRead(RH_ENA)==HIGH){
      encoderRH_Pos++; // CW
    }
    else{
      encoderRH_Pos--; // CCW
    }
  }
  else{ //must be a high-to-low edge on channel B
    //check channel A to see which way encoder is turning
    if(digitalRead(RH_ENA)==LOW){
      encoderRH_Pos++; // CW
    }
    else{
      encoderRH_Pos--; // CCW
    }
  }
  rwheel_msg.data = encoderRH_Pos;
}
