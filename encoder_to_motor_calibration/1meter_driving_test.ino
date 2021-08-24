// TEST FOR DRIVING FORWARD 1 METRE
#include <PID_v1.h>

double Pk1 = 4;
double Ik1 = 4;
double Dk1 = 0.03;
double Setpoint1, Input1, Output1, Output1a; // PID variables 1
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT); // PID setup 1

double Pk2 = 4;
double Ik2 = 4;
double Dk2 = 0.03;
double Setpoint2, Input2, Output2, Output2a; // PID variables 2
PID PID1(&Input2, &Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT); // PID setup 2

float demand1;
float demand2;

unsigned long currentMillis;
unsigned long previousMillis;

// wheel encoder interrupts
#define encoder0PinA 2 // encoder 1 (0)
#define encoder0PinB 3 //           (1)

#define encoder1PinA 18 // encoder 2 (5)
#define encoder1PinB 19 //           (4)

volatile long encoder0Pos = 0; // encoder 1 pos
volatile long encoder1Pos = 0; // encoder 2 pos

void setup() {
  pinMode(4, OUTPUT); // motor PWM pins
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  pinMode(encoder0PinA, INPUT_PULLUP); // encoder pins
  pinMode(encoder0PinB, INPUT_PULLUP);
  
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);

  attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(1, doEncoderB, CHANGE);
  attachInterrupt(4, doEncoderC, CHANGE);
  attachInterrupt(5, doEncoderD, CHANGE);

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
        demand1 = 14500; //how many encoder counts per meter? measured manually
        demand2 = 14500; //how many encoder counts per meter? measured manually
      }
      else if(c=='z'){
        //drive to zero or stop
        demand1 = 0;
        demand2 = 0;
      }
    }
    
    Serial.print(encoder0Pos);
    Serial.print(" , ");
    Serial.println(encoder1Pos);

    Setpoint1 = demand1;
    Input1 = encoder0Pos;
    PID1.Compute();

    Setpoint2 = demand2;
    Input2 = encoder1Pos;
    PID2.Compute();

    //DRIVE MOTOR
    if(Output1>0){
      Output1a = abs(Output1);//CW
      analogWrite(6,Output1a);
      analogWrite(7,0);
    }
    else if(Output1<0){
      Output1a = abs(Output1);//CCW
      analogWrite(6,0);
      analogWrite(7,Output1a);
    }
    else{
      analogWrite(6,0);//NOT MOVING
      analogWrite(7,0);
    }

    //OTHER MOTOR
    if(Output2>0){
      Output2a = abs(Output2);//CW
      analogWrite(5,Output2a);
      analogWrite(4,0);
    }
    else if(Output2<0){
      Output2a = abs(Output2);//CCW
      analogWrite(5,0);
      analogWrite(4,Output2a);
    }
    else{
      analogWrite(5,0);//NOT MOVING
      analogWrite(4,0);
    }
  }
}



//*************************encoders interrupt functions******************
//****************** encoder 1 ********************
void doEncoderA(){
  //look for low-to-high on channel A
  if(digitalRead(encoder0PinA)==HIGH){
    //check channel B to see which way encoder is turning
    if(digitalRead(encoder0PinB)==LOW){
      encoder0Pos++; // CW
    }
    else{
      encoder0Pos--; // CCW
    }
  }
  else{ //must be a high-to-low edge on channel A
    //check channel B to see which way encoder is turning
    if(digitalRead(encoder0PinB)==HIGH){
      encoder0Pos++; // CW
    }
    else{
      encoder0Pos--; // CCW
    }
  }
}

void doEncoderB(){
  //look for low-to-high on channel B
  if(digitalRead(encoder0PinB)==HIGH){
    //check channel A to see which way encoder is turning
    if(digitalRead(encoder0PinA)==HIGH){
      encoder0Pos++; // CW
    }
    else{
      encoder0Pos--; // CCW
    }
  }
  else{ //must be a high-to-low edge on channel B
    //check channel A to see which way encoder is turning
    if(digitalRead(encoder0PinA)==LOW){
      encoder0Pos++; // CW
    }
    else{
      encoder0Pos--; // CCW
    }
  }
}

//****************** encoder 2 ********************
void doEncoderC(){
  //look for low-to-high on channel A
  if(digitalRead(encoder1PinA)==HIGH){
    //check channel B to see which way encoder is turning
    if(digitalRead(encoder1PinB)==LOW){
      encoder0Pos--; // CW
    }
    else{
      encoder0Pos++; // CCW
    }
  }
  else{ //must be a high-to-low edge on channel A
    //check channel B to see which way encoder is turning
    if(digitalRead(encoder1PinB)==HIGH){
      encoder0Pos--; // CW
    }
    else{
      encoder0Pos++; // CCW
    }
  }
}

void doEncoderD(){
  //look for low-to-high on channel B
  if(digitalRead(encoder1PinB)==HIGH){
    //check channel A to see which way encoder is turning
    if(digitalRead(encoder1PinA)==HIGH){
      encoder0Pos--; // CW
    }
    else{
      encoder0Pos++; // CCW
    }
  }
  else{ //must be a high-to-low edge on channel B
    //check channel A to see which way encoder is turning
    if(digitalRead(encoder1PinA)==LOW){
      encoder0Pos--; // CW
    }
    else{
      encoder0Pos++; // CCW
    }
  }
}
