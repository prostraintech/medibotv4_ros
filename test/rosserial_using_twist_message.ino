       
#include <ros.h>
#include <geometry_msgs/Twist.h>
#define LED 13
float lin, ang;

geometry_msgs::Twist vel_msg;
void subscriberCallback(const geometry_msgs::Twist& vel_msg);
ros::NodeHandle node_handle;
ros::Subscriber<geometry_msgs::Twist> vel_subscriber("/cmd_vel", &subscriberCallback);



void setup(){
  pinMode(LED, OUTPUT);
  node_handle.initNode();
  node_handle.subscribe(vel_subscriber);
}

void loop(){ 
  node_handle.spinOnce();
  delay(1);
}

void subscriberCallback(const geometry_msgs::Twist& vel_msg){
  lin = vel_msg.linear.x;
  ang = vel_msg.angular.z;

  if (lin){
    digitalWrite(LED, LOW); 
  }
  else if (ang){
    digitalWrite(LED, LOW); 
  }
  else{
    digitalWrite(LED, HIGH);
  }
}
