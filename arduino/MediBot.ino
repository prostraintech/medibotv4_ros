#include <Esplora.h>

int joystick, movement;

int CS_STT = 42, CS_STP = 41, CS_FWD = 40, CS_RVR = 32, CS_RGT = 31, CS_LFT = 30; // Digital Input Console
int PAN_D1 = 24 ; int PAN_D2 = 23; int TILT_D1 = 22; int TILT_D2 = 2;

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


#define ESTOP 39 


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  pinMode(BRAKE,OUTPUT);pinMode(SW_SEL, INPUT);
  digitalWrite(SW_SEL, HIGH);

  pinMode(CS_STT, INPUT);pinMode(CS_STP, INPUT);pinMode(CS_FWD, INPUT);
  pinMode(CS_RVR, INPUT);pinMode(CS_RGT, INPUT);pinMode(CS_LFT, INPUT);
  digitalWrite(CS_STT, HIGH); digitalWrite(CS_STP, HIGH); digitalWrite(CS_FWD, HIGH);
  digitalWrite(CS_RVR, HIGH); digitalWrite(CS_RGT, HIGH); digitalWrite(CS_LFT, HIGH); 

  pinMode(PAN_D1,OUTPUT);pinMode(PAN_D2,OUTPUT);pinMode(TILT_D1,OUTPUT);pinMode(TILT_D2,OUTPUT);
  
  pinMode(LH_D2,OUTPUT);pinMode(LH_D3,OUTPUT);
  pinMode(RH_D2,OUTPUT);pinMode(RH_D3,OUTPUT);
  pinMode(LH_D1, OUTPUT);pinMode(RH_D1, OUTPUT);
  
  analogWrite(RH_D1,25);analogWrite(LH_D1,25);
  digitalWrite(RH_D2,HIGH);digitalWrite(RH_D3,HIGH);
  digitalWrite(LH_D2,HIGH);digitalWrite(LH_D3,HIGH);

  pinMode(LED_R_LH,OUTPUT);pinMode(LED_G_LH,OUTPUT);pinMode(LED_B_LH,OUTPUT);
  pinMode(LED_R_RH,OUTPUT);pinMode(LED_G_RH,OUTPUT);pinMode(LED_B_RH,OUTPUT);

  pinMode(LSR_out1, INPUT);pinMode(LSR_out2, INPUT);pinMode(LSR_out3, INPUT);
  digitalWrite(LSR_out1, HIGH);digitalWrite(LSR_out2, HIGH);digitalWrite(LSR_out3, HIGH);
  attachInterrupt(digitalPinToInterrupt(43), Lidar_out1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(42), Lidar_out2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(41), Lidar_out3, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ESTOP),EMG_STOP, CHANGE);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  while (digitalRead(SW_SEL) == 0)  // remote mode
  {
    while (Serial.available()>0)
    {
       movement = Serial.parseInt();
       motor(movement);
    }

    motor(10); // stop motor
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
    
    else if(digitalRead(CS_LFT) == 0 && digitalRead(CS_RGT) == 1 && digitalRead(CS_FWD) == 1 && digitalRead(CS_RVR) == 1 && digitalRead(CS_STT) == 0 && digitalRead(CS_STP) == 1)
    {
      //Move pan to the left
      movement = 6;
      motor(movement);
    }

    else if(digitalRead(CS_LFT) == 1 && digitalRead(CS_RGT) == 0 && digitalRead(CS_FWD) == 1 && digitalRead(CS_RVR) == 1 && digitalRead(CS_STT) == 0 && digitalRead(CS_STP) == 1)
    {
      //Move pan to the right
      movement = 7;
      motor(movement);
    }

    else if(digitalRead(CS_LFT) == 1 && digitalRead(CS_RGT) == 1 && digitalRead(CS_FWD) == 0 && digitalRead(CS_RVR) == 1 && digitalRead(CS_STT) == 0 && digitalRead(CS_STP) == 1)
    {
      //Move Tilt Up
      movement = 8;
      motor(movement);
    }
    
    else if(digitalRead(CS_LFT) == 1 && digitalRead(CS_RGT) == 1 && digitalRead(CS_FWD) == 1 && digitalRead(CS_RVR) == 0 && digitalRead(CS_STT) == 0 && digitalRead(CS_STP) == 1)
    {
      //Move Tilt down
      movement = 9;
      motor(movement);
    }
  
    else
    {
      //Stop all motors
      movement = 10;
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

void EMG_STOP(){

    digitalWrite(LED_R_LH,254);digitalWrite(LED_R_RH,254);
    digitalWrite(LED_G_LH,0);digitalWrite(LED_G_RH,0);
    digitalWrite(LED_B_LH,0);digitalWrite(LED_B_RH,0);
    
    digitalWrite(LH_D2,LOW);
    digitalWrite(RH_D2,LOW);

}