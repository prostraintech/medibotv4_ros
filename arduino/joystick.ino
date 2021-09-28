int joystick;

int LH_D2 = 28 ; int LH_D3 = 27 ; int RH_D2 = 26 ; int RH_D3 = 25; 
int LH_D1 = 3; int RH_D1 = 4;
//int ESTOP =39; int BRAKE = 29;

int LED_R_LH = 7 ; int LED_G_LH = 8 ; int LED_B_LH = 9 ;
int LED_R_RH = 10 ; int LED_G_RH = 11 ; int LED_B_RH = 12 ; //Digital Output (PWM)


#define ESTOP 39 


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
//  pinMode(BRAKE,OUTPUT);
//  attachInterrupt(digitalPinToInterrupt(ESTOP),EMG_STOP, HIGH);
  
  pinMode(LH_D2,OUTPUT);pinMode(LH_D3,OUTPUT);
  pinMode(RH_D2,OUTPUT);pinMode(RH_D3,OUTPUT);
  pinMode(LH_D1, OUTPUT);pinMode(RH_D1, OUTPUT);
  
  analogWrite(RH_D1,25);analogWrite(LH_D1,25);
  digitalWrite(RH_D2,HIGH);digitalWrite(RH_D3,HIGH);
  digitalWrite(LH_D2,HIGH);digitalWrite(LH_D3,HIGH);

  pinMode(LED_R_LH,OUTPUT);pinMode(LED_G_LH,OUTPUT);pinMode(LED_B_LH,OUTPUT);
  pinMode(LED_R_RH,OUTPUT);pinMode(LED_G_RH,OUTPUT);pinMode(LED_B_RH,OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  while (Serial.available()>0)
  {
    joystick = Serial.parseInt();


    if (joystick == 1)  // forward
    {
      digitalWrite(RH_D2,HIGH);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(RH_D3,LOW);
      digitalWrite(LH_D3,LOW);
      analogWrite(RH_D1,125);
      analogWrite(LH_D1,125);
      Serial.println("forward");

      digitalWrite(LED_R_LH,0);
      digitalWrite(LED_G_LH,0);
      digitalWrite(LED_B_LH,254);
      digitalWrite(LED_R_RH,0);
      digitalWrite(LED_G_RH,0);
      digitalWrite(LED_B_RH,254);
    }

    else if(joystick == 2)  // reverse
    {
      digitalWrite(RH_D2,HIGH);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(RH_D3,HIGH);
      digitalWrite(LH_D3,HIGH);
      analogWrite(RH_D1,90);
      analogWrite(LH_D1,90);
      Serial.println("reverse");

      digitalWrite(LED_R_LH,0);
      digitalWrite(LED_G_LH,0);
      digitalWrite(LED_B_LH,254);
      digitalWrite(LED_R_RH,0);
      digitalWrite(LED_G_RH,0);
      digitalWrite(LED_B_RH,254);
    }

    else if(joystick == 3)  // right
    {
      digitalWrite(RH_D2,HIGH);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(RH_D3,HIGH);
      digitalWrite(LH_D3,LOW);
      analogWrite(RH_D1,90);
      analogWrite(LH_D1,90);
      Serial.println("right");

      digitalWrite(LED_R_LH,0);
      digitalWrite(LED_G_LH,0);
      digitalWrite(LED_B_LH,254);
      digitalWrite(LED_R_RH,0);
      digitalWrite(LED_G_RH,0);
      digitalWrite(LED_B_RH,254);
    }

    else if(joystick == 4)  // left
    {
      digitalWrite(RH_D2,HIGH);
      digitalWrite(LH_D2,HIGH);
      digitalWrite(RH_D3,LOW);
      digitalWrite(LH_D3,HIGH);
      analogWrite(RH_D1,90);
      analogWrite(LH_D1,90);
      Serial.println("left");

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

  // default stop without instruction from joystick
  analogWrite(RH_D1,25);analogWrite(LH_D1,25);
  digitalWrite(RH_D2,LOW);
  digitalWrite(LH_D2,LOW);
  Serial.println("stop");

  delay(1);
}

//void EMG_STOP()
//{
//
//    digitalWrite(LED_R_LH,254);
//    digitalWrite(LED_G_LH,0);
//    digitalWrite(LED_B_LH,0);
//    digitalWrite(LED_R_RH,254);
//    digitalWrite(LED_G_RH,0);
//    digitalWrite(LED_B_RH,0);
//
//    digitalWrite(LH_D2,LOW);
//    digitalWrite(RH_D2,LOW);
//
//
//}