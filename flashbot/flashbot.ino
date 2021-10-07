

float demandx=0; // in m/s
float demandz=-1; // in rad/s
unsigned long currentMillis;
unsigned long previousMillis;  





void setup() {
  Serial.begin(9600);
}

void loop() {
  currentMillis = millis();
  if(currentMillis - previousMillis >= 10){
    previousMillis = currentMillis;
    Serial.println(WHEEL_SEPARATION);
    Serial.println(WHEEL_DIAMETER);
    Serial.println(WHEEL_RADIUS);
  }


}
