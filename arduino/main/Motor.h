
class Motor{
  public:
    Motor(int, int, int);//pan tilt motor with encoder
    Motor(int, int, int, int, int);//driving motor with encoder
    void Rotate(int, int, int);
    long getEncoderPos();
    int getEncoderA();
    int getEncoderB();
    long doEncoderA();
    long doEncoderB();
  private:
    //motor pins  
    int D1;//pwm value
    int D2;//status, high=move , low=stop
    int D3;//direction, low=forward, high=backward
    //encoder pins
    int ENA;
    int ENB;
    void initMotorPins();
    int protectOutput(int);
    void initEncoderPins();
    volatile long encoder_Pos = 0;
    bool drive; //true=driving , false=pantilt
};

Motor::Motor(int D1, int D2, int ENA){
  this->D1 = D1;
  this->D2 = D2;
  this->drive = false;
  Motor::initMotorPins();
  this->ENA = ENA;
}

Motor::Motor(int D1, int D2, int D3, int ENA, int ENB){
  this->D1 = D1;
  this->D2 = D2;
  this->D3 = D3;
  this->drive = true;
  Motor::initMotorPins();
  this->ENA = ENA;
  this->ENB = ENB;
  //Motor::initEncoderPins();
}

void Motor::initMotorPins(){
  pinMode(this->D1, OUTPUT);
  pinMode(this->D2, OUTPUT);
  if(this->drive) pinMode(this->D3, OUTPUT);
}

void Motor::Rotate(int pwm, int lower_lim=0, int upper_lim=0){
  if(this->drive){
    //Inverted pwm values
    analogWrite(this->D1, 255-abs(pwm));
    //analogWrite(this->D1, abs(pwm));
    digitalWrite(this->D2, pwm!=0);
    digitalWrite(this->D3, pwm<0);
    //digitalWrite(BRAKE,HIGH);
  }
  else{
    if(analogRead(this->ENA)>lower_lim && analogRead(this->ENA)<upper_lim){
      digitalWrite(this->D1, pwm!=0);
    }
    else if(analogRead(this->ENA)==lower_lim && pwm<0){
      digitalWrite(this->D1, pwm!=0);
    }
    else if(analogRead(this->ENA)==upper_lim && pwm>0){
      digitalWrite(this->D1, pwm!=0);
    }
    else{
      digitalWrite(this->D1, 0);
    }
    digitalWrite(this->D2, pwm>0);
  }
}

int Motor::protectOutput(int val){
  // For safety reasons
  (val>150)? val = 150 : val;
  return val;
}

void Motor::initEncoderPins(){
  pinMode(this->ENA, INPUT_PULLUP);
  pinMode(this->ENB, INPUT_PULLUP);
}

long Motor::doEncoderA(){
  if(digitalRead(this->ENA)==HIGH){
    (digitalRead(this->ENB)==LOW)? this->encoder_Pos++ : this->encoder_Pos--;
  }
  else{ 
    (digitalRead(this->ENB)==HIGH)? this->encoder_Pos++ : this->encoder_Pos--;
  }
  return this->encoder_Pos;
}

long Motor::doEncoderB(){
  if(digitalRead(this->ENB)==HIGH){
    (digitalRead(this->ENA)==HIGH)? this->encoder_Pos++ : this->encoder_Pos--;
  }
  else{ 
    (digitalRead(this->ENA)==LOW)? this->encoder_Pos++ : this->encoder_Pos--;
  }
  return this->encoder_Pos;
}

long Motor::getEncoderPos(){
  return this->encoder_Pos;
}

int Motor::getEncoderA(){
  return this->ENA;
}

int Motor::getEncoderB(){
  return this->ENB;
}
