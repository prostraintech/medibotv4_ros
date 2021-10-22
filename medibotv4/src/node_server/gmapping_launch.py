#! /usr/bin/env python

import rospy
import rospkg 
import time
import roslaunch
state_cmd = rospy.get_param('~state_cmd', '')

if __name__ == "__main__":
    rospy.init_node('slam_gmapping_server', log_level=rospy.INFO)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('medibotv4')+"/launch/mapping/gmapping.launch"])
    launch.start()
    rospy.spin() # mantain the service open.
