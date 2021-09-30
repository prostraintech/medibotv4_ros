/**
 * NOTES:
 * 
 * To use the keyboard:
 * 
 * $ roslaunch turtlebot_teleop keyboard_teleop.launch
 * 
 * This node publishes into "/cmd_vel_mux/input/teleop" which has the following
 * format (geometry_msgs/Twist):
 * 
 * geometry_msgs/Vector3 linear: float {x,y,z}
 * geometry_msgs/Vector3 angular: float {x,y,z}
 *       
 * To execute the program correctly:
 * 
 * $ roslaunch turtlebot_teleop keyboard_teleop.launch
 * $ rosrun rosserial_python serial_node.py /dev/ttyACM0
 *
 * http://wiki.ros.org/rosserial_python
 *
 * TODO:
 * - catch CTRL-C and exit cleanly (stopping the robot)
 * - how to obtain the maximum velocity (modify bounds)
 * 
 */
//wheel radius, r = 160mm = 0.16m
//wheel separation, l = 498mm = 0.498m
//lidar height from ground = 229.4mm
//diff drive parameters in metre
#define WHEEL_SEPARATION 0.498
#define WHEEL_RADIUS 0.16
#define MAX_LINEAR 0.2
#define MAX_ANGULAR 1.0

#include "DifferentialDriveRobot.h"

void ddr_callback(const geometry_msgs::Twist&);

DifferentialDriveRobot *my_robot;

ros::NodeHandle nh;	
ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", ddr_callback);

void setup()
{
  my_robot = new DifferentialDriveRobot(new DCMotor(LH_D1, LH_D2, LH_D3),new DCMotor(RH_D1, RH_D2, RH_D3),WHEEL_RADIUS,WHEEL_SEPARATION);

  my_robot->updateParameters(MAX_LINEAR,MAX_ANGULAR);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() 
{  
    nh.spinOnce();
    delay(1);   
}

void ddr_callback(const geometry_msgs::Twist& msg) {

	my_robot->move(msg.linear.x, msg.angular.z);
	
}