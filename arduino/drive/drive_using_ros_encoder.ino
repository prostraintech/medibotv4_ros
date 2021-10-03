#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
ros::NodeHandle  nh;
//motor pins
#define LH_D1 3          // Left hand PWM
#define LH_D2 28         // Left hand STOP
#define LH_D3 27         // Left hand DIRECTION
#define RH_D1 4          // Right hand PWM
#define RH_D2 26         // Right hand STOP
#define RH_D3 25         // Right hand DIRECTION
#define BRAKE 29         // Brake
// wheel encoder pins
#define LH_ENA 46 // left encoder A
#define LH_ENB 45 // left encoder B         
#define RH_ENA 44 // right encoder A
#define RH_ENB 43 // right encoder B 
float demandx=0; // in m/s
float demandz=0; // in rad/s
int pwm = 80; 
int pwm_turn = 60;
volatile long encoderLH_Pos = 0; // encoder left pos
volatile long encoderRH_Pos = 0; // encoder right pos
unsigned long currentMillis;
unsigned long previousMillis;  
double lastCmdVelReceived = 0;
void Move(double LH_PWM, double RH_PWM);

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

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(pwm_sub);
  nh.subscribe(pwm_turn_sub);
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
  if(currentMillis - previousMillis >= 10){
    previousMillis = currentMillis;
    if(demandx>0 && abs(demandz)<=1){//forward
      Move(pwm,pwm);
    }
    if(demandx<0 && abs(demandz)<=1){//reverse
      Move(-pwm,-pwm);
    }
    else if(abs(demandx)<=0.1 && demandz>0){//left
      Move(-pwm_turn,pwm_turn);
    }
    else if(abs(demandx)<=0.1 && demandz<0){//right
      Move(pwm_turn,-pwm_turn);
    }
    else if(demandx==0 && demandz==0){
      Move(0,0);
    }
    lwheel_pub.publish(&lwheel_msg);
    rwheel_pub.publish(&rwheel_msg);
  }

  //Stop the robot if there are no cmd_vel messages
  /*if(millis() - lastCmdVelReceived > 50) {
      analogWrite(LH_D1,0);//left wheel stop
      digitalWrite(LH_D2,LOW);
      analogWrite(RH_D1,0);//right wheel stop
      digitalWrite(RH_D2,LOW);
      digitalWrite(BR,HIGH);
  }*/
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

void Move(double LH_PWM, double RH_PWM){
    if(LH_PWM>0){//left wheel forward
      analogWrite(LH_D1,LH_PWM);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,LOW);
    }
    else if(LH_PWM<0){//left wheel backward
      analogWrite(LH_D1,abs(LH_PWM));
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,HIGH);
    }
    else{//left wheel stop
      analogWrite(LH_D1,0);
      digitalWrite(LH_D2,LOW);
      digitalWrite(BRAKE,HIGH);
    }

    if(RH_PWM>0){//right wheel forward
      analogWrite(RH_D1,RH_PWM);
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,LOW);
    }
    else if(RH_PWM<0){//right wheel backward
      analogWrite(RH_D1,abs(RH_PWM));
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,HIGH);
    }
    else{//right wheel stop
      analogWrite(RH_D1,0);
      digitalWrite(RH_D2,LOW);
      digitalWrite(BRAKE,HIGH);
    }
}