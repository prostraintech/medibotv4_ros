
#define WHEEL_SEPARATION 0.498
#define WHEEL_DIAMETER 0.32
#define WHEEL_RADIUS 0.16
#define LOOPTIME 10
float demandx=0; // in m/s
float demandz=0; // in rad/s
unsigned long currentMillis;
unsigned long previousMillis;  


void setup() {
  Serial.begin(9600);
}

void loop() {
  currentMillis = millis();
  if(currentMillis - previousMillis >= LOOPTIME){
    previousMillis = currentMillis;
    if(Serial.available()>0){
      String c=Serial.readString();
      demandx=c.toFloat();
    }


    float rplusl  = (2*demandx)/WHEEL_RADIUS;
    float rminusl = (demandz*WHEEL_SEPARATION)/WHEEL_RADIUS;
    float right_omega = (rplusl+rminusl)/2;
    float left_omega  = rplusl-right_omega;
    float right_vel = right_omega*WHEEL_RADIUS;
    float left_vel  = left_omega *WHEEL_RADIUS;
    float left_speed = min(((left_vel/1.0)*255),255);
    float right_speed = min(((right_vel/1.0)*255),255);

    Serial.print("left_pwm=");
    Serial.print(left_speed);
    Serial.print(" , right_pwm=");
    Serial.println(right_speed);


  }

}
