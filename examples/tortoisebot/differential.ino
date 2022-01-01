#define WHEEL_SEPARATION 0.498
#define WHEEL_DIAMETER 0.32
#define WHEEL_RADIUS 0.16
#define LOOPTIME 30
float demandx=0.2; // in m/s
float demandz=0; // in rad/s
unsigned long currentMillis;
unsigned long previousMillis;  


// void cmd_vel_callback( const geometry_msgs::Twist& twist){
//   demandx = twist.linear.x;
//   demandz = twist.angular.z;
//   lastCmdVelReceived = millis();
// }

// ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);


void setup() {
  // nh.getHardware()->setBaud(115200);
  // nh.initNode();
  // nh.subscribe(cmd_vel_sub);
  Seria.begin(9600);
}

void loop() {
  nh.spinOnce();
  currentMillis = millis();
  if(currentMillis - previousMillis >= 10){
    previousMillis = currentMillis;
    float rplusl  = (2*demandx)/WHEEL_RADIUS;
    float rminusl = (demandz*WHEEL_SEPARATION)/WHEEL_RADIUS;
    float right_omega = (rplusl+rminusl)/2;
    float left_omega  = rplusl-right_omega;
    float right_vel = right_omega*WHEEL_RADIUS;
    float left_vel  = left_omega *WHEEL_RADIUS;
    float left_speed = min(((left_vel/0.5)*100),100);
    float right_speed = min(((right_vel/0.5)*100),100);

    Serial.print("left_vel=");
    Serial.print(left_vel);
    Serial.print(" , right_vel=");
    Serial.println(right_vel);
    Serial.print("left_speed=");
    Serial.print(left_speed);
    Serial.print(" , right_speed=");
    Serial.println(right_speed);


  }

  //Stop the robot if there are no cmd_vel messages
  /*if(millis() - lastCmdVelReceived > 50) {
      analogWrite(LH_D1,0);//left wheel stop
      digitalWrite(LH_D2,LOW);
      analogWrite(RH_D1,0);//right wheel stop
      digitalWrite(RH_D2,LOW);
      digitalWrite(BR,HIGH);
  }*/
}
