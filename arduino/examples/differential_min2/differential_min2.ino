
#define WHEEL_SEPARATION 0.498
#define WHEEL_DIAMETER 0.32
#define WHEEL_RADIUS 0.16
#define LOOPTIME 10
#define MAX_LINEAR 0.5
#define MAX_ANGULAR 1.0
#define MAX_VALUE 255
#define INT_PWM   uint8_t


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


    float bound_right = (MAX_LINEAR + MAX_ANGULAR * WHEEL_SEPARATION/2.0) / WHEEL_RADIUS;
    float bound_left = (MAX_LINEAR - MAX_ANGULAR * WHEEL_SEPARATION/2.0) / WHEEL_RADIUS;
    float u_r = (demandx + demandz * WHEEL_SEPARATION/2.0) / WHEEL_RADIUS;
    float u_l = (demandx - demandz * WHEEL_SEPARATION/2.0) / WHEEL_RADIUS;
    Serial.print(u_l);
    Serial.print("   ,   ");
    Serial.println(u_r);

    u_r > 0? u_r = map(static_cast<INT_PWM>(u_r), 0, bound_right, 0, MAX_VALUE) : u_r = map(static_cast<INT_PWM>(-u_r), 0, bound_right, 0, MAX_VALUE);
    u_l > 0? u_l = map(static_cast<INT_PWM>(u_l), 0, bound_left, 0, MAX_VALUE) : u_l = map(static_cast<INT_PWM>(-u_l), 0, bound_left, 0, MAX_VALUE);

    Serial.print(u_l);
    Serial.print("   ,   ");
    Serial.println(u_r);

    u_r > MAX_VALUE? u_r = MAX_VALUE : u_r;
    u_l > MAX_VALUE? u_l = MAX_VALUE : u_l;

    Serial.print(u_l);
    Serial.print("   ,   ");
    Serial.println(u_r);
    Serial.println("----------------------------------");



  }

}
