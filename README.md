# medibotv4_ros
IIUM Medibot V4 ROS packages and firmware code.

## Software required:
1. Ubuntu 18.04
2. ROS Melodic Morenia

## Hardware required:
1. Advantech MIC-710AIX / Jetson Xavier NX (robot computer)
2. Arduino Due (base controller)
3. Hokuyo UST-05LA (lidar)
4. 2 x Motors with Encoders

## Installation:
Clone the repository
- ```cd ~/catkin_ws/src```
- ```git clone https://github.com/zulhafiz-zulkifli/medibotv4_ros.git```
Install all packages dependencies
- ```rosdep install medibotv4 -y```
Build the packages
- ```cd ~/catkin_ws```
- ```catkin_make```
- ```source ~/catkin_ws/devel/setup.bash```