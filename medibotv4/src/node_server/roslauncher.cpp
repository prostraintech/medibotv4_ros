#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

    void callback(std_msgs::String msg){
        std::stringstream ss;
        //ss << "roslaunch " << msg.data;
        if(msg.data=="start_gmapping"){
            ss << "rosrun medibotv4 gmapping_launch.py";
            std::system(ss.str().c_str());//std::system only accept c_str() a.k.a char*
        }
        else if(msg.data=="start_navigation"){
            ss << "rosrun medibotv4 navigation_launch.py";
            std::system(ss.str().c_str());//std::system only accept c_str() a.k.a char*
        }   
    }

    int main(int argc, char** argv){
        ros::init(argc,argv,"roslaunch_launcher");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("/launch", 10, callback);
        ros::spin();
        return 0;
    }