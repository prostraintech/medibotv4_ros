#! /usr/bin/env python
import numpy
import rospy
import rospkg 
import time
import string
from medibotv4.srv import *
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray

class WaypointControl(object):
    def __init__(self):
        self._waypoints = ["spotA", "spotB", "spotC", "spotD"]
        # _robot_status can be either moving or reached.
        self._robot_status = "moving";
        self._mb_result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult , self.mb_result_sub_callback)

    def mb_result_sub_callback(self, msg):
        if msg.status.status == 3: # if robot has reached goal
            self._robot_status = "reached"



if __name__ == "__main__":
    rospy.init_node('waypoint_control_node', log_level=rospy.INFO) 
    wpc = WaypointControl()
    Ncycle = 1
    start_time = rospy.get_time()
    rospy.wait_for_service('/spots/send_goal')
    while not rospy.is_shutdown():
        for label in wpc._waypoints:
            # wait for send goal service
            send_goal_service_client = rospy.ServiceProxy('/spots/send_goal', SendGoal)
            msg = SendGoalRequest()
            msg.label = label
            # check if going to first spot
            if label == wpc._waypoints[0]:
                print("\n------- TEST " + str(Ncycle) + "-------")
                start_time = rospy.get_time()
            # call service to send goal to a waypoint
            response = send_goal_service_client(msg)
            print("Moving to " + label)
            # is robot still moving? if yes wait
            while wpc._robot_status != "reached":
                pass
            # reached at a spot
            print("Reached at " + label + ".\n")
            wpc._robot_status = "moving"
            # check if reached at final spot
            if label == wpc._waypoints[-1]:
                Ncycle += 1
                print("Time taken = " + str(round(rospy.get_time()-start_time,2)) + " seconds")