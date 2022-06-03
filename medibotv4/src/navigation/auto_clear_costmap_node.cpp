#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "clear_costmap_service");
	ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    std::string tf_prefix;

    nh_priv.param<std::string>("tf_prefix",tf_prefix,"");
	if(tf_prefix != ""){
		tf_prefix = "/"+tf_prefix;
	}

	ROS_INFO("Waiting for clear costmap service...");
	ros::service::waitForService(tf_prefix+"/move_base/clear_costmaps");
	ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>(tf_prefix+"/move_base/clear_costmaps");
    std_srvs::Empty srv;
    ROS_INFO("Clearing costmap periodically...");

	while(ros::ok()){
		clearClient.call(srv);
		ROS_INFO("Clearing Costmap...");
		ros::Duration(10).sleep();
	}
}