//ENCODER READING TEST
#define LOOPTIME 10
// wheel encoder pins
#define LH_ENA 46 // left encoder A
#define LH_ENB 45 // left encoder B         
#define RH_ENA 44 // right encoder A
#define RH_ENB 43 // right encoder B   
unsigned long currentMillis;
unsigned long previousMillis;      
volatile long encoderLH_Pos = 0; // encoder left pos
volatile long encoderRH_Pos = 0; // encoder right pos

void setup() {
  pinMode(LH_ENA, INPUT_PULLUP); // encoder pins
  pinMode(LH_ENB, INPUT_PULLUP);
  pinMode(RH_ENA, INPUT_PULLUP);
  pinMode(RH_ENB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LH_ENA), doEncoderLHA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LH_ENB), doEncoderLHB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENA), doEncoderRHA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENB), doEncoderRHB, CHANGE);
  Serial.begin(115200);
}

void loop() {
  currentMillis = millis();
  if(currentMillis - previousMillis >= LOOPTIME){
    previousMillis = currentMillis;
    Serial.print(encoderLH_Pos);
    Serial.print(" , ");
    Serial.println(encoderRH_Pos);
  }
}


//*************************encoders interrupt functions******************
//****************** encoder left ********************
void doEncoderLHA(){
  if(digitalRead(LH_ENA)==HIGH){
    if(digitalRead(LH_ENB)==LOW){
      encoderLH_Pos++; // CW
    }
    else{
      encoderLH_Pos--; // CCW
    }
  }
  else{ 
    if(digitalRead(LH_ENB)==HIGH){
      encoderLH_Pos++; // CW
    }
    else{
      encoderLH_Pos--; // CCW
    }
  }
}
void doEncoderLHB(){
  if(digitalRead(LH_ENB)==HIGH){
    if(digitalRead(LH_ENA)==HIGH){
      encoderLH_Pos++; // CW
    }
    else{
      encoderLH_Pos--; // CCW
    }
  }
  else{ 
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
  if(digitalRead(RH_ENA)==HIGH){
    if(digitalRead(RH_ENB)==LOW){
      encoderRH_Pos++; // CW
    }
    else{
      encoderRH_Pos--; // CCW
    }
  }
  else{ 
    if(digitalRead(RH_ENB)==HIGH){
      encoderRH_Pos++; // CW
    }
    else{
      encoderRH_Pos--; // CCW
    }
  }
}
void doEncoderRHB(){
  if(digitalRead(RH_ENB)==HIGH){
    if(digitalRead(RH_ENA)==HIGH){
      encoderRH_Pos++; // CW
    }
    else{
      encoderRH_Pos--; // CCW
    }
  }
  else{ 
    if(digitalRead(RH_ENA)==LOW){
      encoderRH_Pos++; // CW
    }
    else{
      encoderRH_Pos--; // CCW
    }
  }
}
