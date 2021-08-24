// TEST FOR DRIVING at various velocities
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
float demandx;
float demandz;

unsigned long currentMillis;
unsigned long previousMillis;

// wheel encoder interrupts
#define encoder0PinA 2 // encoder 1 (0)
#define encoder0PinB 3 //           (1)

#define encoder1PinA 18 // encoder 2 (5)
#define encoder1PinB 19 //           (4)

volatile long encoder0Pos = 0; // encoder 1 pos
volatile long encoder1Pos = 0; // encoder 2 pos

float encoder0Diff;
float encoder1Diff;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;

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
        //0.5 m/s test
        demandx = 0.5; //desired velocity in m/s for each wheel
        demandz = 0; //desired velocity in m/s for each wheel
      }
      else if(c=='b'){
        //0.25 m/s test
        demandx = 0.25; //desired velocity in m/s for each wheel
        demandz = 0; //desired velocity in m/s for each wheel
      }
      else if(c=='c'){
        demandx = 0; //turn at 1 rad/s
        demandz = 1; 
      }
      else if(c=='d'){
        demandx = 0; //turn at 1 rad/s
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
    demand1 = demandx - (demandz*0.145);
    demand2 = demandx + (demandz*0.145);
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

    encoder0Diff = encoder0Pos - encoder0Prev; // work out difference from last time
    encoder1Diff = encoder1Pos - encoder1Prev; // this is the current speed in counts per 10ms

    encoder0Error = (demand1*129) - encoder0Diff;
    encoder1Error = (demand2*129) - encoder1Diff;

    encoder0Prev = encoder0Pos; // bookmark previous values
    encoder1Prev = encoder1Pos;

    Setpoint1 = demand1*129; //drive wheel 1 at 129 counts per 10ms cycle
    Input1 = encoder0Diff;
    PID1.Compute();

    Setpoint2 = demand2*129; //drive wheel 2 at 129 counts per 10ms cycle
    Input2 = encoder1Diff; //drive
    PID2.Compute();
    
    Serial.print(Setpoint2);
    Serial.print(" , ");
    Serial.println(encoder1Diff);
    Serial.print(" , ");
    Serial.println(encoder1Pos);
    Serial.print(" , ");
    Serial.println(Output2);



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
