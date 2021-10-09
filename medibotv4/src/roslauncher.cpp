#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

    void callback(std_msgs::String msg){
        std::stringstream ss;
        ss << "roslaunch " << msg.data;
        std::system(ss.str().c_str());//std::system only accept c_str() a.k.a char*
    }

    int main(int argc, char** argv){
        ros::init(argc,argv,"roslaunch_launcher");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("/launch", 1, callback);
        ros::spin();
        return 0;
    }