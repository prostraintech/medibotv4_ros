// TEST FOR AT VARIOUS WHEEL(S) VELOCITIES
#include <PID_v1.h>

double Pk1 = 500;
double Ik1 = 0;
double Dk1 = 0;
double Setpoint1, Input1, Output1, Output1a; // PID variables 1
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT); // PID setup 1

double Pk2 = 500;
double Ik2 = 0;
double Dk2 = 0;
double Setpoint2, Input2, Output2, Output2a; // PID variables 2
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT); // PID setup 2

float demand1=0;
float demand2=0;

unsigned long currentMillis;
unsigned long previousMillis;

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

volatile long encoderLH_Pos = 0; // encoder left pos
volatile long encoderRH_Pos = 0; // encoder right pos
float encoder0Diff;
float encoder1Diff;
float encoder0Error;
float encoder1Error;
float encoder0Prev;
float encoder1Prev;

void setup() {
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
  if(currentMillis - previousMillis >= 10){
    previousMillis = currentMillis;

    if(Serial.available()>0){ // manual control of wheels via terminal
      char c = Serial.read();
      if(c=='e'){
        //0.5 m/s test
        demand1 = 0.5; //desired velocity in m/s for each wheel
        demand2 = 0.5; //desired velocity in m/s for each wheel
      }
      else if(c=='w'){
        //0.25 m/s test
        demand1 = 0.25; //desired velocity in m/s for each wheel
        demand2 = 0.25; //desired velocity in m/s for each wheel
      }
      else if(c=='s'){
        //0.1 m/s test
        demand1 = -0.25; //desired velocity in m/s for each wheel
        demand2 = -0.25; //desired velocity in m/s for each wheel
      }
      else if(c=='a'){
        //turn at 1 rad/s
        demand1 = -0.145; 
        demand2 = 0.145; 
      }
      else if(c=='d'){
        //turn at -1 rad/s
        demand1 = 0.145; 
        demand2 = -0.145; 
      }
      else if(c=='z'){
        //stop motors
        demand1 = 0; 
        demand2 = 0; 
      }
      else if(c=='x'){
        demand1 = 0.1;
        demand2 = 0.1;
      }
    }

    encoder0Diff = encoderLH_Pos - encoder0Prev; // work out difference from last time
    encoder1Diff = encoderRH_Pos - encoder1Prev; // this is the current speed in counts per 10ms

    encoder0Error = (demand1*0.97) - encoder0Diff;
    encoder1Error = (demand2*0.97) - encoder1Diff;

    encoder0Prev = encoderLH_Pos; // bookmark previous values
    encoder1Prev = encoderRH_Pos;

    Setpoint1 = demand1*0.97;
    Input1 = encoder0Diff;
    PID1.Compute();

    Setpoint2 = demand2*0.97;
    Input2 = encoder1Diff;
    PID2.Compute();

    Serial.print(Setpoint2);
    Serial.print(" , ");
    Serial.print(encoder1Diff);
    Serial.print(" , ");
    Serial.print(encoderRH_Pos);
    Serial.print(" , ");
    Serial.println(Output2);

    //DRIVE MOTOR
    if(Output1>0){
      Output1a = abs(Output1);//left wheel forward
      //Output1a = map(Output1a, 0, 100, 0, 255)
      analogWrite(LH_D1,Output1a);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,LOW);
    }
    else if(Output1<0){
      Output1a = abs(Output1);//left wheel backward
      //Output1a = map(Output1a, 0, 100, 0, 255)
      analogWrite(LH_D1,abs(Output1a));
      digitalWrite(LH_D2,HIGH);
      digitalWrite(LH_D3,HIGH);
    }
    else{
      analogWrite(LH_D1,0);//left wheel stop
      digitalWrite(LH_D2,LOW);
      //digitalWrite(BR,HIGH);
    }

    //OTHER MOTOR
    if(Output2>0){
      Output2a = abs(Output2);//right wheel forward
      //Output2a = map(Output2a, 0, 100, 0, 255)
      analogWrite(RH_D1,Output2a);
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,LOW);
    }
    else if(Output2<0){//right wheel backward
      Output2a = abs(Output2);
      //Output2a = map(Output2a, 0, 100, 0, 255)
      analogWrite(RH_D1,Output2a);
      digitalWrite(RH_D2,HIGH);
      digitalWrite(RH_D3,HIGH);
    }
    else{
      analogWrite(RH_D1,0);//right wheel stop
      digitalWrite(RH_D2,LOW);
      //digitalWrite(BR,HIGH);
    }
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
