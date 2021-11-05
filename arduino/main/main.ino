//------------------------------------------------------------------------------//
bool USING_ROS_PWM = true;  
bool USING_ROS_DIFF = false; 
//------------------------------------------------------------------------------//
//----------------------------ROS INITIALIZATION--------------------------------//

//#define USE_USBCON
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <medibotv4/SensorState.h>
#include <PID_v1.h>
ros::NodeHandle nh;
int pwm = 60, pwm_turn = 40;
double demandx = 0, demandz = 0, lastCmdVelReceived = 0;
void cmd_vel_callback(const geometry_msgs::Twist& twist);
void pwm_callback(const std_msgs::Int16& pwm_msg);
void pwm_turn_callback(const std_msgs::Int16& pwm_turn_msg);
void pwm_control_callback(const std_msgs::Bool& pwm_control_msg);
std_msgs::Int16 lwheel_msg;
std_msgs::Int16 rwheel_msg;
medibotv4::SensorState sensor_state_msg;
ros::Publisher lwheel_pub("lwheel_ticks", &lwheel_msg);
ros::Publisher rwheel_pub("rwheel_ticks", &rwheel_msg);
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);
ros::Subscriber<std_msgs::Int16> pwm_sub("pwm", pwm_callback);
ros::Subscriber<std_msgs::Int16> pwm_turn_sub("pwm_turn", pwm_turn_callback);
ros::Subscriber<std_msgs::Bool> pwm_control_sub("pwm_control", pwm_control_callback);

//----------------------------GENERAL INITIALIZATION--------------------------------//

#include "Config.h"
#include "Motor.h"
#include "LED.h"
unsigned long currentMillis, previousMillis;
float Ldiff, Lerror, Lprev, Rdiff, Rerror, Rprev;
double Linput, Loutput, Lsetpoint;
double Rinput, Routput, Rsetpoint;
PID LH_pid(&Linput, &Loutput, &Lsetpoint, LH_KP, LH_KI, LH_KD, DIRECT);
PID RH_pid(&Rinput, &Routput, &Rsetpoint, RH_KP, RH_KI, RH_KD, DIRECT);
Motor LH_motor(LH_D1, LH_D2, LH_D3, LH_ENA, LH_ENB);
Motor RH_motor(RH_D1, RH_D2, RH_D3, RH_ENA, RH_ENB);
Motor PAN_motor(PAN_D1, PAN_D2, PAN_EN);
Motor TILT_motor(TILT_D1, TILT_D2, TILT_EN);
LED LH_led(LED_R_LH, LED_G_LH, LED_B_LH);
LED RH_led(LED_R_RH, LED_G_RH, LED_B_RH);
void Move(int lpwm, int rpwm);
void LH_ISRA();
void LH_ISRB();
void RH_ISRA();
void RH_ISRB();
void EMG_STOP();
//----------------------------------------------------------------------------------//


void setup(){
  pinMode(SW_MODE, INPUT_PULLUP);
  pinMode(ESTOP, INPUT_PULLUP);
  pinMode(LSR1,INPUT_PULLUP);
  pinMode(LSR2,INPUT_PULLUP);
  pinMode(LSR3,INPUT_PULLUP);
  pinMode(CS_FWD, INPUT_PULLUP);
  pinMode(CS_RVR, INPUT_PULLUP);
  pinMode(CS_LFT, INPUT_PULLUP);
  pinMode(CS_RGT, INPUT_PULLUP);
  pinMode(CS_STT, INPUT_PULLUP);
  pinMode(CS_STP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LH_motor.getEncoderA()), LH_ISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LH_motor.getEncoderB()), LH_ISRB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_motor.getEncoderA()), RH_ISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_motor.getEncoderB()), RH_ISRB, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ESTOP), EMG_STOP, HIGH);
  ////////////ROS SECTION////////////
  if(USING_ROS_PWM ||USING_ROS_DIFF){
    nh.getHardware()->setBaud(115200);
    nh.initNode('arduino_node');
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(pwm_sub);
    nh.subscribe(pwm_turn_sub);
    nh.subscribe(pwm_control_sub);
    nh.advertise(lwheel_pub);
    nh.advertise(rwheel_pub);
    nh.advertise(sensor_state_pub);
    LH_pid.SetMode(AUTOMATIC);
    LH_pid.SetOutputLimits(-MAX_PWM, MAX_PWM);
    LH_pid.SetSampleTime(1);
    RH_pid.SetMode(AUTOMATIC);
    RH_pid.SetOutputLimits(-MAX_PWM, MAX_PWM);
    RH_pid.SetSampleTime(1);
  }
}

void loop(){
  nh.spinOnce();
  currentMillis = millis();
  if(currentMillis - previousMillis >= LOOPTIME){
    previousMillis = currentMillis;



    //RESCUE MODE
    if(!digitalRead(SW_MODE)){
      if(!digitalRead(CS_FWD) && digitalRead(CS_RVR) && digitalRead(CS_LFT) && digitalRead(CS_RGT) && digitalRead(CS_STT) && digitalRead(CS_STP)){
        Move(MOTOR_SPEED, MOTOR_SPEED);//forward
      }
      else if(digitalRead(CS_FWD) && !digitalRead(CS_RVR) && digitalRead(CS_LFT) && digitalRead(CS_RGT) && digitalRead(CS_STT) && digitalRead(CS_STP)){
        Move(-MOTOR_SPEED, -MOTOR_SPEED);//reverse
      }
      else if(digitalRead(CS_FWD) && digitalRead(CS_RVR) && !digitalRead(CS_LFT) && digitalRead(CS_RGT) && digitalRead(CS_STT) && digitalRead(CS_STP)){
        Move(-DIFF_MOTOR_SPEED, DIFF_MOTOR_SPEED);//left
      }
      else if(digitalRead(CS_FWD) && digitalRead(CS_RVR) && digitalRead(CS_LFT) && !digitalRead(CS_RGT) && digitalRead(CS_STT) && digitalRead(CS_STP)){
        Move(DIFF_MOTOR_SPEED, -DIFF_MOTOR_SPEED);//right
      }
      else if(digitalRead(CS_FWD) && digitalRead(CS_RVR) && !digitalRead(CS_LFT) && digitalRead(CS_RGT) && !digitalRead(CS_STT) && digitalRead(CS_STP)){
        PAN_motor.Rotate(1, PAN_LEFT_LIM, PAN_RIGHT_LIM);//pan left
      }
      else if(digitalRead(CS_FWD) && digitalRead(CS_RVR) && digitalRead(CS_LFT) && !digitalRead(CS_RGT) && !digitalRead(CS_STT) && digitalRead(CS_STP)){
        PAN_motor.Rotate(-1, PAN_LEFT_LIM, PAN_RIGHT_LIM);//pan right
      }
      else if(!digitalRead(CS_FWD) && digitalRead(CS_RVR) && digitalRead(CS_LFT) && digitalRead(CS_RGT) && digitalRead(CS_STT) && !digitalRead(CS_STP)){
        TILT_motor.Rotate(1, TILT_UP_LIM, TILT_DOWN_LIM);//tilt up
      }
      else if(digitalRead(CS_FWD) && !digitalRead(CS_RVR) && digitalRead(CS_LFT) && digitalRead(CS_RGT) && digitalRead(CS_STT) && !digitalRead(CS_STP)){
        TILT_motor.Rotate(-1, TILT_UP_LIM, TILT_DOWN_LIM);//tilt down
      }
      else{
        Move(0, 0);//stop
        PAN_motor.Rotate(0);//stop pan
        TILT_motor.Rotate(0);//stop tilt
      }
    }

    //REMOTE MODE
    if(digitalRead(SW_MODE)){
      ////////////ROS SECTION////////////////
      if(USING_ROS_PWM){
        if(demandx>0 && abs(demandz)<1){
          Move(pwm, pwm);//forward
        }
        if(demandx<0 && abs(demandz)<1){
          Move(-pwm, -pwm);//reverse
        }
        else if(demandz>0 && abs(demandx)<0.5){
          Move(-pwm_turn, pwm_turn);//left
        }
        else if(demandz<0 && abs(demandx)<0.5){
          Move(pwm_turn, -pwm_turn);//right
        }
        else if(demandx==0 && demandz==0){
          Move(0, 0);//stop
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
        // Lerror = (left_vel*0.95) - Ldiff;
        // Rerror = (right_vel*0.95) - Rdiff;
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
    }
    //Publishing data to ROS
    lwheel_msg.data = LH_motor.getEncoderPos();
    lwheel_pub.publish(&lwheel_msg);
    rwheel_msg.data = RH_motor.getEncoderPos();
    rwheel_pub.publish(&rwheel_msg);
    sensor_state_msg.ir1 = analogRead(IR1);
    sensor_state_msg.ir2 = analogRead(IR2);
    sensor_state_msg.ir3 = analogRead(IR3);
    sensor_state_msg.sonar = analogRead(SON1);
    sensor_state_msg.cliff = analogRead(CSENS);
    sensor_state_msg.laser1 = digitalRead(LSR1);
    sensor_state_msg.laser2 = digitalRead(LSR2);
    sensor_state_msg.laser3 = digitalRead(LSR3);
    sensor_state_msg.switch_mode = digitalRead(SW_MODE);
    sensor_state_msg.estop = digitalRead(ESTOP);
    sensor_state_msg.cs_stt = digitalRead(CS_STT);
    sensor_state_msg.cs_stp = digitalRead(CS_STP);
    sensor_state_msg.cs_fwd = digitalRead(CS_FWD);
    sensor_state_msg.cs_rvr = digitalRead(CS_RVR);
    sensor_state_msg.cs_lft = digitalRead(CS_LFT);
    sensor_state_msg.cs_rgt = digitalRead(CS_RGT);
    sensor_state_pub.publish(&sensor_state_msg);
    //Stop the robot if there are no cmd_vel messages
    if(millis() - lastCmdVelReceived > CMD_VEL_TIMEOUT){
      Move(0, 0);
      PAN_motor.Rotate(0);//stop pan
      TILT_motor.Rotate(0);//stop tilt
    }
  }
}


////////////FUNCTION DEFINITIONS////////////////

void Move(int lpwm, int rpwm){
  LH_motor.Rotate(-lpwm);
  RH_motor.Rotate(rpwm);
  if(lpwm==0 && rpwm==0){
    LH_led.Emit('w');RH_led.Emit('w');
  }
  else{
    LH_led.Emit('b');RH_led.Emit('b');
  }
}

void LH_ISRA(){
  LH_motor.doEncoderA();
}

void LH_ISRB(){
  LH_motor.doEncoderB();
}

void RH_ISRA(){
  RH_motor.doEncoderA();
}

void RH_ISRB(){
  RH_motor.doEncoderB();
}

void EMG_STOP(){
  PAN_motor.Rotate(0);
  TILT_motor.Rotate(0);
  Move(0, 0);
  LH_led.Emit('r');RH_led.Emit('r');
}

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
