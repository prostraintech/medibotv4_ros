int IR1 = A0;
int IR2 = A1;
int IR3 = A2;
int SON1 = A3;
int PAN_EN = A4;
int TILT_EN = A5;
int CSENS = A6; //Analog Input
int LH_ENA = 44; // try git
int LH_ENB = 43;
int RH_ENA = 46;
int RH_ENB = 45; //Digital input Encoder
int CS_STT = 66;
int CS_STP = 67;
int CS_FWD = 68;
int CS_RVR = 69;
int CS_RGT = 49;
int CS_LFT = 48; // Digital Input Console
int LSR_Out1 = 42; // free Zone (outside)
int LSR_Out2 = 41; // caution zone (middle)
int LSR_Out3 = 40; // danger zone (inside)
int ESTOP = 39;
int SW_Mode = 38; // Digital Input General

int BRAKE = 29;
int LH_D2 = 26;
int LH_D3 = 25;
int RH_D2 = 28;
int RH_D3 = 27;
int PAN_D1 = 24;
int PAN_D2 = 23;
int TILT_D1 = 2;
int TILT_D2 = 3; // Digital Output
int LH_D1 = 5;
int RH_D1 = 4;
int LED_R_LH = 10;
int LED_G_LH = 11;
int LED_B_LH = 12;
int LED_R_RH = 7;
int LED_G_RH = 8;
int LED_B_RH = 9; //Digital Output (PWM)

#define LH_ENA 44
#define LH_ENB 43
#define RH_ENA 46
#define RH_ENB 45
#define ESTOP 39
#define MOTOR_SPEED 50 //400RPM
#define DIFF_MOTOR_SPEED 200 //150RPM

int count = 0;
int debug_speed = 0;
int heartbeat =0;
int reset_order =0;
int permitup=0, permitdown=0, permitright=0, permitleft=0;

volatile unsigned int encoder_RH = 0;
volatile unsigned int encoder_LH = 0;

void setup()
{

  pinMode(LH_ENA, INPUT);
  pinMode(LH_ENB, INPUT);
  pinMode(RH_ENA, INPUT);
  pinMode(RH_ENB, INPUT);
  pinMode(SW_Mode, INPUT);
  pinMode(ESTOP, INPUT);
  pinMode(LSR_Out1,INPUT);
  pinMode(LSR_Out2,INPUT);
  pinMode(LSR_Out3,INPUT);
  digitalWrite(LH_ENA, HIGH);
  digitalWrite(LH_ENB, HIGH);
  digitalWrite(RH_ENA, HIGH);
  digitalWrite(RH_ENB, HIGH);
  digitalWrite(SW_Mode, HIGH);
  digitalWrite(ESTOP, HIGH);
  digitalWrite(LSR_Out1,HIGH);
  digitalWrite(LSR_Out2,HIGH);
  digitalWrite(LSR_Out3,HIGH);

  pinMode(CS_STT, INPUT);
  pinMode(CS_STP, INPUT);
  pinMode(CS_FWD, INPUT);
  pinMode(CS_RVR, INPUT);
  pinMode(CS_RGT, INPUT);
  pinMode(CS_LFT, INPUT);
  digitalWrite(CS_STT, HIGH);
  digitalWrite(CS_STP, HIGH);
  digitalWrite(CS_FWD, HIGH);
  digitalWrite(CS_RVR, HIGH);
  digitalWrite(CS_RGT, HIGH);
  digitalWrite(CS_LFT, HIGH);

  attachInterrupt(digitalPinToInterrupt(RH_ENA), Encoder_RH_ENA, CHANGE); //Pin 2
  attachInterrupt(digitalPinToInterrupt(LH_ENA), Encoder_LH_ENA, CHANGE); //Pin 18
  attachInterrupt(digitalPinToInterrupt(ESTOP), EMG_STOP, HIGH);

  pinMode(BRAKE, OUTPUT);
  pinMode(LH_D2, OUTPUT);
  pinMode(LH_D3, OUTPUT);
  pinMode(RH_D2, OUTPUT);
  pinMode(RH_D3, OUTPUT);
  pinMode(PAN_D1, OUTPUT);
  pinMode(PAN_D2, OUTPUT);
  pinMode(TILT_D1, OUTPUT);
  pinMode(TILT_D2, OUTPUT);
  pinMode(LH_D1, OUTPUT);
  pinMode(RH_D1, OUTPUT);
  pinMode(LED_R_LH, OUTPUT);
  pinMode(LED_G_LH, OUTPUT);
  pinMode(LED_B_LH, OUTPUT);
  pinMode(LED_R_RH, OUTPUT);
  pinMode(LED_G_RH, OUTPUT);
  pinMode(LED_B_RH, OUTPUT);

  analogWrite(RH_D1, 229);
  analogWrite(LH_D1, 229);
  digitalWrite(BRAKE, HIGH);

  Serial.begin(9600);
  digitalWrite(RH_D2, LOW);
  digitalWrite(RH_D3, LOW);
  digitalWrite(LH_D2, LOW);
  digitalWrite(LH_D3, LOW);
}

void set_led_right(int r, int g, int b)
{
  digitalWrite(LED_R_LH, 255 - r);
  digitalWrite(LED_G_LH, 255 - g);
  digitalWrite(LED_B_LH, 255 - b);
}

void set_led_left(int r, int g, int b)
{
  digitalWrite(LED_R_RH, 255 - r);
  digitalWrite(LED_G_RH, 255 - g);
  digitalWrite(LED_B_RH, 255 - b);
}

void loop()
{

/*-------+
+  LED   +
+--------*/

  set_led_right(255, 255, 0); //RGB
  set_led_left(255, 255, 0);  //RGB

/*-------+
+ DEBUG  +
+--------*/

  debug_speed++;
  if (debug_speed % 20 == 0)
  {

    Serial.print("Mode  = ");
    Serial.println(digitalRead(SW_Mode));
    Serial.print("Estop  = ");
    Serial.println(digitalRead(ESTOP));

    Serial.print("CS Start,STOP,LH,RH,FWD,REV = ");
    Serial.print(digitalRead(CS_STT));
    Serial.print(digitalRead(CS_STP));
    Serial.print(digitalRead(CS_LFT));
    Serial.print(digitalRead(CS_RGT));
    Serial.print(digitalRead(CS_FWD));
    Serial.println(digitalRead(CS_RVR));

    int Analog_IR1 = analogRead(IR1);
    int Analog_IR2 = analogRead(IR2);
    int Analog_IR3 = analogRead(IR3);
    Serial.print("IR(FWD) = ");
    Serial.println(Analog_IR1);
    Serial.print("IR (RH) = ");
    Serial.println(Analog_IR2);
    Serial.print("IR (LH) = ");
    Serial.println(Analog_IR3);
    Serial.print("SON = ");
    Serial.println(analogRead(SON1));
    Serial.print("CSense = ");
    Serial.println(analogRead(CSENS));
    Serial.print("Pan Angle = ");
    Serial.println(analogRead(PAN_EN));
    Serial.print("Tilt Angle = ");
    Serial.println(analogRead(TILT_EN));
    Serial.print("Encoder_LH = ");
    Serial.println(encoder_LH);
    Serial.print("Encoder_RH = ");
    Serial.println(encoder_RH);
    //Serial.print("Count "); Serial.println(count);
    Serial.println("");

    //prevent overflow
    if (debug_speed == 15000)
      debug_speed = 0;
  }

/*-------+
+  NOTE  +
+--------+

1 - MOVE FORWARD
2 - MOVE REVERSE
3 - MOVE RIGHT
4 - MOVE LEFT
5 - PAN LEFT
6 - PAN RIGHT
7 - TILT UP
8 - TITL DOWN

/*-------+
+ RESCUE +
+--------*/

  if (digitalRead(SW_Mode) == 0)
  { // 0 = Rescue

    // Serial.println("Rescue Mode");

    if (digitalRead(CS_LFT) == 1 && digitalRead(CS_RGT) == 0 && digitalRead(CS_STT) == 1 && digitalRead(CS_STP) == 1)
    {
      move(3);
    }
    else if (digitalRead(CS_LFT) == 0 && digitalRead(CS_RGT) == 1 && digitalRead(CS_STT) == 1 && digitalRead(CS_STP) == 1)
    {
      move(4);
    }
    else if (digitalRead(CS_LFT) == 1 && digitalRead(CS_RGT) == 1 && digitalRead(CS_FWD) == 0 && digitalRead(CS_RVR) == 1 && digitalRead(CS_STT) == 1 && digitalRead(CS_STP) == 1)
    {
      move(1);
    }
    else if (digitalRead(CS_LFT) == 1 && digitalRead(CS_RGT) == 1 && digitalRead(CS_FWD) == 1 && digitalRead(CS_RVR) == 0 && digitalRead(CS_STT) == 1 && digitalRead(CS_STP) == 1)
    {
      move(2);
    }
    else if (digitalRead(CS_LFT) == 0 && digitalRead(CS_RGT) == 1 && digitalRead(CS_FWD) == 1 && digitalRead(CS_RVR) == 1 && digitalRead(CS_STT) == 0 && digitalRead(CS_STP) == 1)
    {
       if(analogRead(PAN_EN) < 350) 
      {
        permitleft = 0;
        permitright = 1;
      
      }

      else
      {
        permitleft = 1;
      }

      move(5);
    }
    else if (digitalRead(CS_LFT) == 1 && digitalRead(CS_RGT) == 0 && digitalRead(CS_FWD) == 1 && digitalRead(CS_RVR) == 1 && digitalRead(CS_STT) == 0 && digitalRead(CS_STP) == 1)
    {
      if(analogRead(PAN_EN) > 550) 
      {
       // Serial.println("here");
        permitright = 0;
        permitleft = 1;
      
      }

      else
      {
        permitright = 1;
      }

      move(6);
    }
    else if (digitalRead(CS_LFT) == 1 && digitalRead(CS_RGT) == 1 && digitalRead(CS_FWD) == 0 && digitalRead(CS_RVR) == 1 && digitalRead(CS_STT) == 1 && digitalRead(CS_STP) == 0)
    {

      Serial.print("permitup ="); Serial.println(permitup);

      if(analogRead(TILT_EN) < 455) 
      {
       // Serial.println("here");
        permitup = 0;
        permitdown = 1;
      
      }

      else
      {
        permitup = 1;
      }

        move(7);
    

      
    }
    else if (digitalRead(CS_LFT) == 1 && digitalRead(CS_RGT) == 1 && digitalRead(CS_FWD) == 1 && digitalRead(CS_RVR) == 0 && digitalRead(CS_STT) == 1 && digitalRead(CS_STP) == 0)
    {

       if(analogRead(TILT_EN) > 550) 
      {
        permitdown = 0;
        permitup = 1;
      
      }

      else
      {
        permitdown = 1;
      }

    
        move(8);
    

      
    }

    else
    {
      move(0);
    }

    

      
  }

/*-------+
+ REMOTE +
+--------*/

  if (digitalRead(SW_Mode) == 1)
  { // 1 = Remote
  
    // Serial.println("Remote Mode");
    /*if (digitalRead(LSR_Out2) ==1 ){
       //reset_order =0;
       Serial.println("obstacle detected");
      }
      else if (digitalRead(LSR_Out2) == 0)
      {
        Serial.println("no obstacle");
      }*/
    
    while (Serial.available() > 0)
    {
      int order = Serial.parseInt();

    
       if(analogRead(TILT_EN) > 550) 
      {
        permitdown = 0;
        permitup = 1;
      
      }

      else
      {
        permitdown = 1;
      }

      if(analogRead(TILT_EN) < 455) 
      {
       // Serial.println("here");
        permitup = 0;
        permitdown = 1;
      
      }

      else
      {
        permitup = 1;
      }

      ///for pan limit
       if(analogRead(PAN_EN) < 350) 
      {
        permitleft = 0;
        permitright = 1;
      
      }

      else
      {
        permitleft = 1;
      }

      if(analogRead(PAN_EN) > 550) 
      {
       // Serial.println("here");
        permitright = 0;
        permitleft = 1;
      
      }

      else
      {
        permitright = 1;
      }



      if(digitalRead(LSR_Out2)==1 && order!=9 && reset_order ==0){   // This consider object detected. Inlcude reset joystick 
         order =0;        
         }
      else if (digitalRead(LSR_Out2)==1 && order ==9 && reset_order ==0){
        reset_order = 1;
         }
      else if(digitalRead(LSR_Out3) == 1){
        order =0;
      }
      if (digitalRead(LSR_Out2)==0){
       reset_order =0;
       Serial.println("no obstacle");
      }

       else if (digitalRead(LSR_Out2) == 1)
      {
        Serial.println("yes obstacle");
      }
      
      
      heartbeat =0;
      move(order);
     
     }

      

  heartbeat++;
    if (heartbeat>300){
      move(0);
    }

     

    
       
  }

  delay(100); // delay in between reads for stability
  count++;
}

/*-------+
+ E_STOP +
+-------*/

void EMG_STOP()
{

  set_led_right(255, 0, 0); //RGB - Red
  set_led_left(255, 0, 0);  //RGB - Red

  digitalWrite(TILT_D1, LOW);
  digitalWrite(PAN_D1, LOW);
  digitalWrite(LH_D2, LOW);
  digitalWrite(RH_D2, LOW);
}

/*--------+
+ ENCODER +
+---------*/

void Encoder_RH_ENA()
{
  if (digitalRead(RH_ENB) == digitalRead(RH_ENA))
  {
    encoder_RH = encoder_RH - 1; //you may need to redefine positive and negative directions
  }
  else
  {
    encoder_RH = encoder_RH + 1;
  }
}

void Encoder_LH_ENA()
{
  if (digitalRead(LH_ENB) == digitalRead(LH_ENA))
  {
    encoder_LH = encoder_LH - 1; //you may need to redefine positive and negative directions
  }
  else
  {
    encoder_LH = encoder_LH + 1;
  }
}

/*-----------------+
+ MOTOR, PAN, TILT +
+------------------*/

void move(int order)
{
  if (order == 1)
  {
    //Move forward
      digitalWrite(RH_D2, HIGH);
      digitalWrite(LH_D2, HIGH);
      digitalWrite(RH_D3, LOW);
      digitalWrite(LH_D3, HIGH);
      analogWrite(RH_D1, MOTOR_SPEED);
      analogWrite(LH_D1, MOTOR_SPEED);
    //  Serial.print("Move Forward");Serial.print("\n");
    // analogWrite(LED_FRT,100);
  }

  else if (order == 2)
  {
      //move reverse
      digitalWrite(RH_D2, HIGH);
      digitalWrite(LH_D2, HIGH);
      digitalWrite(RH_D3, HIGH);
      digitalWrite(LH_D3, LOW);
      analogWrite(RH_D1, MOTOR_SPEED);
      analogWrite(LH_D1, MOTOR_SPEED);
      // analogWrite(LED_FRT,100);
      //  Serial.print("Move Reverse");Serial.print("\n");
  }

  else if (order == 3)
  {
      //Move differential to the right
      digitalWrite(RH_D2, HIGH);
      digitalWrite(LH_D2, HIGH);
      digitalWrite(RH_D3, HIGH);
      digitalWrite(LH_D3, HIGH);
      analogWrite(RH_D1, DIFF_MOTOR_SPEED);
      analogWrite(LH_D1, DIFF_MOTOR_SPEED);
      //  analogWrite(LED_FRT,100);
      // Serial.print("Move Left");Serial.print("\n");
  }

  else if (order == 4)
  {
       //Move differential to the left
      digitalWrite(RH_D2, HIGH);
      digitalWrite(LH_D2, HIGH);
      digitalWrite(RH_D3, LOW);
      digitalWrite(LH_D3, LOW);
      analogWrite(RH_D1, DIFF_MOTOR_SPEED);
      analogWrite(LH_D1, DIFF_MOTOR_SPEED);
      //  analogWrite(LED_FRT,100);
      // Serial.print("Move Right");Serial.print("\n");   
  }

  else if (order == 5)
  {
      //Move pan to the left
        if (permitleft == 1)
    {
      digitalWrite(PAN_D1, HIGH);
      digitalWrite(PAN_D2, HIGH);
      digitalWrite(TILT_D1, LOW);
      //  Serial.print("Pan Left");Serial.print("\n"); 
    }

    else if (permitleft== 0)
    {
      digitalWrite(PAN_D1, LOW);
      digitalWrite(PAN_D2, HIGH);
      digitalWrite(TILT_D1, LOW);
    }  
  }

  else if (order == 6)
  {
      //move pan to the right
      if (permitright == 1)
    {
      digitalWrite(PAN_D1, HIGH);
      digitalWrite(PAN_D2, LOW);
      digitalWrite(TILT_D1, LOW);
    }
      //  Serial.print("Pan Right");Serial.print("\n");

      else if (permitright == 0)
      {
      digitalWrite(PAN_D1, LOW);
      digitalWrite(PAN_D2, LOW);
      digitalWrite(TILT_D1, LOW);
      }
  }

  else if (order == 7)
  {
    if (permitup == 1)
    {
      //Move tilt up
      digitalWrite(TILT_D1, HIGH);
      digitalWrite(TILT_D2, HIGH);
      digitalWrite(PAN_D1, LOW);
    }

    else if (permitup == 0)
    {
      //Move tilt up stop
      digitalWrite(TILT_D1, LOW);
      digitalWrite(TILT_D2, HIGH);
      digitalWrite(PAN_D1, LOW);
    }
      //  Serial.print("Tilt Up");Serial.print("\n");
  }

    else if (order == 8)
  {
      if (permitdown == 1)
    {
      //Move tilt down
      digitalWrite(TILT_D1, HIGH);
      digitalWrite(TILT_D2, LOW);
      digitalWrite(PAN_D1, LOW);
      //  Serial.print("Tilt Down");Serial.print("\n");
    }

    else if (permitdown == 0)
    {
      //Move tilt down stop
      digitalWrite(TILT_D1, LOW);
      digitalWrite(TILT_D2, LOW);
      digitalWrite(PAN_D1, LOW);
      //  Serial.print("Tilt Down");Serial.print("\n");
    }
  }

  else if (order == 0)
  {
      //Stop all motors
      digitalWrite(TILT_D1, LOW);
      digitalWrite(PAN_D1, LOW);
      digitalWrite(LH_D2, LOW);
      digitalWrite(RH_D2, LOW);
      analogWrite(RH_D1, 229);
      analogWrite(LH_D1, 229);
  }

    else
  {
    Serial.println("Error: order Not understood");
  }

  
}