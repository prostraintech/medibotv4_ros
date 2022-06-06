#! /usr/bin/env python
import numpy
import rospy
import rospkg 
import time
import string
from move_base_msgs.msg import MoveBaseActionGoal
from std_srvs.srv import Empty, EmptyRequest

robot_namespace = rospy.get_param("~robot_ns", "")
if robot_namespace != "":
    robot_namespace = "/" + robot_namespace 

def sub_callback(msg):
    clear_costmap_service_client = rospy.ServiceProxy(robot_namespace+'/move_base/clear_costmaps', Empty)
    clear_costmap_service_request = EmptyRequest()
    clear_costmap_service_response = clear_costmap_service_client(clear_costmap_service_request)


if __name__ == "__main__":
    rospy.init_node('clear_costmap_node', log_level=rospy.INFO)    
    _ = rospy.Subscriber(robot_namespace+'/move_base/goal', MoveBaseActionGoal, sub_callback)
    rospy.spin() # mantain the service open.