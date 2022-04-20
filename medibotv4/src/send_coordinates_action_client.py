#! /usr/bin/env python

import rospy
from kasimx_localization.srv import SaveSpotServiceMessage, SaveSpotServiceMessageResponse
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseResult, MoveBaseGoal
import actionlib
import time
import os
import rosparam


class SendCoordinates(object):
    def __init__(self, label):
        
        client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rate = rospy.Rate(1)

        goal=MoveBaseGoal()
        goal_tmp = Pose()
        
        self._ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
        
        tag = label
        
        while not self._ctrl_c:
    
            goal_tmp.position.x=rosparam.get_param(tag+'/position/x')
            goal_tmp.position.y=rosparam.get_param(tag+'/position/y')
            goal_tmp.position.z=rosparam.get_param(tag+'/position/z')
            goal_tmp.orientation.x=rosparam.get_param(tag+'/orientation/x')
            goal_tmp.orientation.y=rosparam.get_param(tag+'/orientation/y')
            goal_tmp.orientation.z=rosparam.get_param(tag+'/orientation/z')
            goal_tmp.orientation.w=rosparam.get_param(tag+'/orientation/w')
            goal.target_pose.pose=goal_tmp
            goal.target_pose.header.frame_id='map'
            
            client.wait_for_server()
            client.send_goal(goal, feedback_cb=self.callback)
            client.wait_for_result()
            result=client.get_state()
                
            #print result
            if result==3:
                print('successfuly reached point')
                self.shutdownhook()
                
            
    def shutdownhook(self):
            
        rospy.loginfo("shutdown time!")
        self._ctrl_c = True
        
    def callback(self, data):
        return 


if __name__ == "__main__":
    rospy.init_node('send_coordinates_node', log_level=rospy.INFO) 
    #send_coordinates_object = SendCoordinates("table")
    rospy.spin() # mantain the service open.