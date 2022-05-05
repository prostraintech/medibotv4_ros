#! /usr/bin/env python

import rospy
from medibotv4.srv import SaveSpotService, SaveSpotServiceResponse
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatus, GoalID
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseResult, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerResponse
import actionlib
import time
import os
import rosparam


class SendCoordinates(object):
    def __init__(self, label):
        
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        _ = rospy.Subscriber("/move_base/cancel", GoalID, self.cancel_client) 
        _ = rospy.Service('/get_coordinates/reset', Trigger, self.reset) 
        rate = rospy.Rate(1)

        goal = MoveBaseGoal()
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
            
            self.client.wait_for_server()
            self.client.send_goal(goal, feedback_cb=self.feedback_cb)
            self.client.wait_for_result()
            result=self.client.get_state()
                
            #print result
            if result==3:
                print('GOAL REACHED SUCCESSFULLY!')
                self.shutdownhook()
                
            
    def shutdownhook(self):
        rospy.loginfo("shutdown time!")
        self._ctrl_c = True
        
    # def done_cb(self, status, result):
    #     pass

    # def active_cb(self):
    #     pass

    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        pass

    def cancel_client(self, msg):
        self.client.cancel_goal()
        self.shutdownhook()

    def reset(self, request):
        response = TriggerResponse()
        self.client.cancel_goal()
        self.client.wait_for_result()
        self.shutdownhook()
        return response

if __name__ == "__main__":
    rospy.init_node('send_coordinates_node', log_level=rospy.INFO) 
    #send_coordinates_object = SendCoordinates("table")
    rospy.spin() # mantain the service open.