//wheel radius, r = 160mm = 0.16m
//wheel separation, l = 498mm = 0.498m
//lidar height from ground = 229.4mm
#include <PID_v1.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
ros::NodeHandle  nh;
//diff drive parameters in metre
#define WHEEL_SEPARATION 0.498
#define WHEEL_RADIUS 0.16
#define ENCODER_CPM 12900
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
#define BR = 29         // Brake
// wheel encoder pins
#define LH_ENA 46 // left encoder A
#define LH_ENB 45 // left encoder B         
#define RH_ENA 44 // right encoder A
#define RH_ENB 43 // right encoder B   
//velocity PID
double Pk1 = 1, Ik1 = 0, Dk1 = 0.01;
double Setpoint1, Input1, Output1, Output1a; // PID variables 1
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT); // PID setup 1
double Pk2 = 1, Ik2 = 0, Dk2 = 0.01;
double Setpoint2, Input2, Output2, Output2a; // PID variables 2
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT); // PID setup 2

float demand_left;
float demand_right;
float demandx=0; // in m/s
float demandz=0; // in rad/s

unsigned long currentMillis;
unsigned long previousMillis;      

volatile long encoderLH_Pos = 0; // encoder left pos
volatile long encoderRH_Pos = 0; // encoder right pos
float encoderLH_Diff;
float encoderRH_Diff;
float encoderLH_Error;
float encoderRH_Error;
float encoderLH_Prev;
float encoderRH_Prev;

void cmd_vel_callback( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_callback );
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_act_right = 0;                    //Command speed for left wheel in m/s 

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);
  pinMode(RH_D1,OUTPUT); // motor PWM pins
  pinMode(RH_D2,OUTPUT);
  pinMode(RH_D3,OUTPUT);
  pinMode(LH_D1,OUTPUT);
  pinMode(LH_D2,OUTPUT);
  pinMode(LH_D3,OUTPUT);
  pinMode(LH_ENA, INPUT_PULLUP); // encoder pins
  pinMode(LH_ENB, INPUT_PULLUP);
  pinMode(RH_ENA, INPUT_PULLUP);
  pinMode(RH_ENB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LH_ENA), doEncoderLHA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LH_ENB), doEncoderLHB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENA), doEncoderRHA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENB), doEncoderRHB, CHANGE);

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-100,100);
  PID1.SetSampleTime(10);
  PID2.SetMode(AUTOMATIC);
  PID2.SetOutputLimits(-100,100);
  PID2.SetSampleTime(10);

  Serial.begin(115200);
}

void loop() {
  currentMillis = millis();
  if(currentMillis - previousMillis >= LOOPTIME){
    previousMillis = currentMillis;
     if(Serial.available()>0){ // manual control of wheels via terminal
       char c = Serial.read();
       if(c=='a'){
         //0.5 m/s test
         demandx = -0.25; 
         demandz = 0; 
       }
       else if(c=='b'){
         //0.25 m/s test
         demandx = 0.25; 
         demandz = 0; 
       }
       else if(c=='c'){
         demandx = 0; //turn at 1 rad/s
         demandz = 1; 
       }
       else if(c=='d'){
         demandx = 0; //turn at -1 rad/s
         demandz = -1; 
       }
       else if(c=='e'){
         //drive at 0.25m/s and turn at 1 rad/s
         demandx = 0.25; 
         demandz = 1; 
       }
       else if(c=='f'){
         //the other way
         demandx = 0.25; 
         demandz = -1; 
       }
       else if(c=='z'){
         //stop motors
         demandx = 0; 
         demandz = 0; 
       }
     }

    //calculate the two values for differential drive of each wheel
    demand_left = demandx - (demandz*WHEEL_SEPARATION/2); 
    demand_right = demandx + (demandz*WHEEL_SEPARATION/2); 
    /*  There are 12900 encoder counts in one metre
     *  That is 129 per 10 millisecond loop at 1 metre/second velocity
     * 
     *  Demand in metres/second but we need to convert to encoder counts per 10ms loop
     * 
     *  Distance between wheels is 290mm, half of that is 145mm
     *  Circumference of 290mm circle is 910mm, to turn 180' (pi radians), each wheel needs to drive half of that, which is 455,,
     *  To turn one radian, each wheel needs to drive 455/pi = 145mm (per second for 1 rad/s)
     */

    // work out difference in encoder counts per loop

    encoderLH_Diff = encoderLH_Pos - encoderLH_Prev; // work out difference from last time
    encoderRH_Diff = encoderRH_Pos - encoderRH_Prev; // this is the current speed in counts per 10ms

    speed_act_left = encoderLH_Diff/(ENCODER_CPM/100);                    
    speed_act_right = encoderRH_Diff/(ENCODER_CPM/100); 

    encoderLH_Error = (demand_left*(ENCODER_CPM/100)) - encoderLH_Diff;// 12900 ticks in 1m = 129 ticks in 10ms, due to the 10 millis loop
    encoderRH_Error = (demand_right*(ENCODER_CPM/100)) - encoderRH_Diff;

    encoderLH_Prev = encoderLH_Pos; // bookmark previous values
    encoderRH_Prev = encoderRH_Pos;

    Setpoint1 = demand_left*(ENCODER_CPM/100); //drive wheel 1 at 129 counts per 10ms cycle
    Setpoint2 = demand_right*(ENCODER_CPM/100); //drive wheel 2 at 129 counts per 10ms cycle
    Input1 = encoderLH_Diff;
    Input2 = encoderRH_Diff; 
    PID1.Compute();
    PID2.Compute();
    
    Serial.print(Setpoint2);
    Serial.print(" , ");
    Serial.print(encoderRH_Diff);
    Serial.print(" , ");
    Serial.print(encoderRH_Pos);
    Serial.print(" , ");
    Serial.println(Output2);

    //DRIVE MOTOR
    if(Output1>0){
      Output1a = abs(Output1);//left wheel forward
      //Output1a = map(Output1a, 0, 100, 0, 255)
      analogWrite(LH_D1,Output1a);
      digitalWrite(LH_D2,LOW);
      digitalWrite(LH_D3,HIGH);
    }
    else if(Output1<0){
      Output1a = abs(Output1);//left wheel backward
      //Output1a = map(Output1a, 0, 100, 0, 255)
      analogWrite(LH_D1,Output1a);
      digitalWrite(LH_D2,LOW);
      digitalWrite(LH_D3,LOW);
    }
    else{
      analogWrite(LH_D1,0);//left wheel stop
      digitalWrite(LH_D2,HIGH);
      //digitalWrite(BR,HIGH);
    }

    //OTHER MOTOR
    if(Output2>0){
      Output2a = abs(Output2);//right wheel forward
      //Output2a = map(Output2a, 0, 100, 0, 255)
      analogWrite(RH_D1,Output2a);
      digitalWrite(RH_D2,LOW);
      digitalWrite(RH_D3,HIGH);
    }
    else if(Output2<0){//right wheel backward
      Output2a = abs(Output2);
      //Output2a = map(Output2a, 0, 100, 0, 255)
      analogWrite(RH_D1,Output2a);
      digitalWrite(RH_D2,LOW);
      digitalWrite(RH_D3,LOW);
    }
    else{
      analogWrite(RH_D1,0);//right wheel stop
      digitalWrite(RH_D2,HIGH);
      //digitalWrite(BR,HIGH);
    }
  }
  publishSpeed(LOOPTIME);
  nh.spinOnce();
}

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
//  nh.loginfo("Publishing odometry");
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
}
