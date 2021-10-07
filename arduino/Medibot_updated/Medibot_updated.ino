
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
ros::NodeHandle  nh;
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
void LED(int LH_R, int LH_G, int LH_B, int RH_R, int RH_G, int RH_B);
void lidar (int state);
void motor(int movement);

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


int joystick, movement;

int CS_STT = 42, CS_STP = 41, CS_FWD = 40, CS_RVR = 32, CS_RGT = 31, CS_LFT = 30;           // Digital Input Console

int LH_D2 = 28, LH_D3 = 27, RH_D2 = 26, RH_D3 = 25, LH_D1 = 3, RH_D1 = 4;
int ESTOP = 39, BRAKE = 29, SW_SEL = 38;

int LED_R_LH = 7, LED_G_LH = 8, LED_B_LH = 9 ;
int LED_R_RH = 10, LED_G_RH = 11, LED_B_RH = 12 ; //Digital Output (PWM)

const int LSR_out1 = 49;                  // Lidar STOP zone
const int LSR_out2 = 48;                  // Lidar SLOW zone
const int LSR_out3 = 47;                  // Lidar FREE zone
volatile unsigned int lid_1 = 0;          // to store lidar 1 value
volatile unsigned int lid_2 = 0;          // to store lidar 2 value
volatile unsigned int lid_3 = 0;          // to store lidar 3 value


void setup() {
  //ROS SETUP
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(pwm_sub);
  nh.subscribe(pwm_turn_sub);
  nh.advertise(lwheel_pub);
  nh.advertise(rwheel_pub);
  pinMode(LH_ENA, INPUT_PULLUP); // encoder pins
  pinMode(LH_ENB, INPUT_PULLUP);
  pinMode(RH_ENA, INPUT_PULLUP);
  pinMode(RH_ENB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LH_ENA), doEncoderLHA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LH_ENB), doEncoderLHB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENA), doEncoderRHA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENB), doEncoderRHB, CHANGE);

  // put your setup code here, to run once:

  pinMode(BRAKE,OUTPUT);pinMode(SW_SEL, INPUT);
  digitalWrite(SW_SEL, HIGH);

  pinMode(CS_STT, INPUT);pinMode(CS_STP, INPUT);pinMode(CS_FWD, INPUT);
  pinMode(CS_RVR, INPUT);pinMode(CS_RGT, INPUT);pinMode(CS_LFT, INPUT);
  digitalWrite(CS_STT, HIGH); digitalWrite(CS_STP, HIGH); digitalWrite(CS_FWD, HIGH);
  digitalWrite(CS_RVR, HIGH); digitalWrite(CS_RGT, HIGH); digitalWrite(CS_LFT, HIGH); 
  
  pinMode(LH_D2,OUTPUT);pinMode(LH_D3,OUTPUT);
  pinMode(RH_D2,OUTPUT);pinMode(RH_D3,OUTPUT);
  pinMode(LH_D1, OUTPUT);pinMode(RH_D1, OUTPUT);
  
  // analogWrite(RH_D1,25);analogWrite(LH_D1,25);
  // digitalWrite(RH_D2,HIGH);digitalWrite(RH_D3,HIGH);
  // digitalWrite(LH_D2,HIGH);digitalWrite(LH_D3,HIGH);

  pinMode(LED_R_LH,OUTPUT);pinMode(LED_G_LH,OUTPUT);pinMode(LED_B_LH,OUTPUT);
  pinMode(LED_R_RH,OUTPUT);pinMode(LED_G_RH,OUTPUT);pinMode(LED_B_RH,OUTPUT);

  pinMode(LSR_out1, INPUT);pinMode(LSR_out2, INPUT);pinMode(LSR_out3, INPUT);
  digitalWrite(LSR_out1, HIGH);digitalWrite(LSR_out2, HIGH);digitalWrite(LSR_out3, HIGH);
  // attachInterrupt(digitalPinToInterrupt(43), Lidar_out1, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(42), Lidar_out2, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(41), Lidar_out3, CHANGE);
  
}

void loop() {
  while (digitalRead(SW_SEL) == 0)  // remote mode
  {
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

  while (digitalRead(SW_SEL) == 1)  // rescue mode
  {
    if (digitalRead(CS_LFT) == 1 && digitalRead(CS_RGT) == 1 && digitalRead(CS_FWD) == 0 && digitalRead(CS_RVR) == 1 && digitalRead(CS_STT) == 1 && digitalRead(CS_STP) == 1 ) 
    {
      //Move forward
      movement = 1;
      motor(movement);
    }

    else if(digitalRead(CS_LFT) == 1 && digitalRead(CS_RGT) == 1 && digitalRead(CS_FWD) == 1 && digitalRead(CS_RVR) == 0 && digitalRead(CS_STT) == 1 && digitalRead(CS_STP) == 1)
    {
      //move reverse
      movement = 3;
      motor(movement);    
    }
    
    else if(digitalRead(CS_LFT) == 1 && digitalRead(CS_RGT) == 0 && digitalRead(CS_FWD) == 1 && digitalRead(CS_RVR) == 1 && digitalRead(CS_STT) == 1 && digitalRead(CS_STP) == 1)
    {
      //Move differential to the right
      movement = 4;
      motor(movement);  
    }
    
    else if(digitalRead(CS_LFT) == 0 && digitalRead(CS_RGT) == 1 && digitalRead(CS_FWD) == 1 && digitalRead(CS_RVR) == 1 && digitalRead(CS_STT) == 1 && digitalRead(CS_STP) == 1)
    {
      //Move differential to the left
      movement = 5;
      motor(movement);
    }
  
    else
    {
      //Stop all motors
      movement = 6;
      motor(movement);
    }
  }

  delay(1);
}

void Lidar_out1() {
  if (digitalRead(LSR_out1))
  { 
    lid_1 = 1;  // if object detected
    lidar(lid_1);
  }

  else
  { 
    lid_1 = 0;  // if object not detected
  }
}

void Lidar_out2() {
  if (digitalRead(LSR_out1))
  { 
    lid_2 = 1;  // if object detected
    lidar(lid_2);
  }

  else
  { 
    lid_2 = 0;  // if object not detected
  }
}

void Lidar_out3() {
  if (digitalRead(LSR_out1))
  { 
    lid_3 = 1;  // if object detected
    lidar(lid_3);
  }

  else
  { 
    lid_3 = 0;  // if object not detected
  }
}


void motor(int movement){
  if (movement == 1 || movement == 2 || movement == 3 || movement == 4)
  {
    if (movement == 1)  // forward
    {
      digitalWrite(RH_D2,HIGH);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(RH_D3,LOW);
      digitalWrite(LH_D3,LOW);
      analogWrite(RH_D1,125);
      analogWrite(LH_D1,125);
      Serial.println("forward");
    }

    else if (movement == 2)  // forward slow
    {
      digitalWrite(RH_D2,HIGH);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(RH_D3,LOW);
      digitalWrite(LH_D3,LOW);
      analogWrite(RH_D1,90);
      analogWrite(LH_D1,90);
      Serial.println("forward slow");
    }

    else if(movement == 3)  // reverse
    {
      digitalWrite(RH_D2,HIGH);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(RH_D3,HIGH);
      digitalWrite(LH_D3,HIGH);
      analogWrite(RH_D1,90);
      analogWrite(LH_D1,90);
      Serial.println("reverse");
    }

    else if(movement == 4)  // right
    {
      digitalWrite(RH_D2,HIGH);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(RH_D3,HIGH);
      digitalWrite(LH_D3,LOW);
      analogWrite(RH_D1,90);
      analogWrite(LH_D1,90);
      Serial.println("right");
    }

    else if(movement == 5)  // left
    {
      digitalWrite(RH_D2,HIGH);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(RH_D3,LOW);
      digitalWrite(LH_D3,HIGH);
      analogWrite(RH_D1,90);
      analogWrite(LH_D1,90);
      Serial.println("left");
    }

    digitalWrite(LED_R_LH,0);
    digitalWrite(LED_G_LH,0);
    digitalWrite(LED_B_LH,254);
    digitalWrite(LED_R_RH,0);
    digitalWrite(LED_G_RH,0);
    digitalWrite(LED_B_RH,254);
    
  }
  
  else 
  {
  //Stop all motors
    digitalWrite(LH_D2,LOW);
    digitalWrite(RH_D2,LOW);
    analogWrite(RH_D1,25);
    analogWrite(LH_D1,25);
    Serial.println("stop");

    digitalWrite(LED_R_LH,254);
    digitalWrite(LED_G_LH,254);
    digitalWrite(LED_B_LH,254);
    digitalWrite(LED_R_RH,254);
    digitalWrite(LED_G_RH,254);
    digitalWrite(LED_B_RH,254);
  }
}

void lidar (int state){
  if (lid_1 == 1 && lid_2 == 0 && lid_3 == 0)
  {
    motor(movement); // regular movement
  }
  else if (lid_1 == 1 && lid_2 == 1 && lid_3 == 0)
  {
    movement = 2;
    motor(movement);  // slow
  }
  else if (lid_1 == 1 && lid_2 == 1 && lid_3 == 1)
  {
    movement = 6;
    motor(movement); // stop
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

void Move(double LH_PWM, double RH_PWM){
    if(LH_PWM>0){//left wheel forward
      analogWrite(LH_D1,LH_PWM);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,LOW);
      LED(0,0,254,0,0,254);
    }
    else if(LH_PWM<0){//left wheel backward
      analogWrite(LH_D1,abs(LH_PWM));
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,HIGH);
      LED(0,0,254,0,0,254);
    }
    else{//left wheel stop
      analogWrite(LH_D1,0);
      digitalWrite(LH_D2,LOW);
      digitalWrite(BRAKE,HIGH);
      LED(254,254,254,254,254,254);
    }

    if(RH_PWM>0){//right wheel forward
      analogWrite(RH_D1,RH_PWM);
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,LOW);
      LED(0,0,254,0,0,254);
    }
    else if(RH_PWM<0){//right wheel backward
      analogWrite(RH_D1,abs(RH_PWM));
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,HIGH);
      LED(0,0,254,0,0,254);
    }
    else{//right wheel stop
      analogWrite(RH_D1,0);
      digitalWrite(RH_D2,LOW);
      digitalWrite(BRAKE,HIGH);
      LED(254,254,254,254,254,254);
    }
}

void LED(int LH_R, int LH_G, int LH_B, int RH_R, int RH_G, int RH_B){
      digitalWrite(LED_R_LH,LH_R);
      digitalWrite(LED_G_LH,LH_G);
      digitalWrite(LED_B_LH,LH_B);
      digitalWrite(LED_R_RH,RH_R);
      digitalWrite(LED_G_RH,RH_G);
      digitalWrite(LED_B_RH,RH_B);
}
