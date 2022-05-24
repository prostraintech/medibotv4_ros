#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "clear_costmap_service");
	ros::NodeHandle nh;
	ROS_INFO("Waiting for clear costmap service...");
	ros::service::waitForService("/move_base/clear_costmaps");
	ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty srv;
    ROS_INFO("Clearing costmap periodically...");

	while(ros::ok()){
		clearClient.call(srv);
		ROS_INFO("Clearing Costmap...");
		ros::Duration(3).sleep();
	}
}