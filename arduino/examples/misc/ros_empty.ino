#define USE_USBCON
#include <ros.h>
ros::NodeHandle  nh;

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
}

void loop() {
  nh.spinOnce();
}


