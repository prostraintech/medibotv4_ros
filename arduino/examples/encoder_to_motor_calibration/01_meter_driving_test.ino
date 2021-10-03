// TEST FOR DRIVING FORWARD 1 METRE
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
PID PID1(&Input2, &Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT); // PID setup 2

float demand1;
float demand2;

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
      if(c=='a'){
        //drive 1 meter test
        demand1 = 97; //how many encoder counts per meter? measured manually
        demand2 = 97; //how many encoder counts per meter? measured manually
      }
      else if(c=='z'){
        //drive to zero or stop
        demand1 = 0;
        demand2 = 0;
      }
    }
    
    Serial.print(encoderLH_Pos);
    Serial.print(" , ");
    Serial.println(encoderRH_Pos);

    Setpoint1 = demand1;
    Input1 = encoderLH_Pos;
    PID1.Compute();

    Setpoint2 = demand2;
    Input2 = encoderRH_Pos;
    PID2.Compute();

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
