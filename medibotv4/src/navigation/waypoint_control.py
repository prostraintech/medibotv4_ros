#! /usr/bin/env python
import numpy
import rospy
import rospkg 
import time
import string
from medibotv4.srv import *
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray, GoalID
from calculate_distance_traveled import CalculateDistanceTraveled
from std_srvs.srv import Empty, EmptyRequest


class WaypointControl(object):
    def __init__(self, waypoints=["spotA", "spotB","spotC", "spotD"]):
        self.waypoints = waypoints
        self.robot_status = "moving"; # can be either moving or reached.
        self.mb_result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult , self.mb_result_sub_callback)
        self.mb_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.Ntest = 1
        self.start_time = 0.0
        self.end_time = 0.0
        self.total_distance = 0.0
        #cdt = None
        rospy.on_shutdown(self.shutdownhook)
        rospy.wait_for_service('/spots/send_goal') # wait for send goal service

    def mb_result_sub_callback(self, msg):
        if msg.status.status == 3: # if robot has reached goal
            self.robot_status = "reached"

    def shutdownhook(self):
        msg = GoalID()
        self.mb_cancel_pub.publish(msg)

    def begin(self):
        for i in self.waypoints:
            cdt = CalculateDistanceTraveled() # start calculating distance traveled
            clear_costmap_service_client = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_costmap_service_request = EmptyRequest()
            send_goal_service_client = rospy.ServiceProxy('/spots/send_goal', SendGoal)
            send_goal_service_request = SendGoalRequest()
            send_goal_service_request.label = i
            # check if going to first spot in the list
            if i == self.waypoints[0]:
                print("\n--------- TEST " + str(self.Ntest) + "---------")
            # call service to send goal to a waypoint
            clear_costmap_service_response = clear_costmap_service_client(clear_costmap_service_request)
            send_goal_service_response = send_goal_service_client(send_goal_service_request)
            print("  Moving to " + i)
            # wait until the goal is reached
            while self.robot_status != "reached":
                pass
            # reached at a spot
            print("  Reached at " + i + ".\n")
            self.robot_status = "moving"
            # check if reached at first spot in the list
            if i == self.waypoints[0]:
                self.start_time = rospy.get_time() # start the timer
            else:
                self.total_distance += cdt.getTotalDistance()
            # check if reached at final spot in the list
            if i == self.waypoints[-1]:
                self.Ntest += 1
                self.end_time = rospy.get_time() # end the timer
                print("  Time taken = " + str(round(self.end_time-self.start_time,2)) + " seconds")
                print("  Total Distance Traveled = " + str(self.total_distance) + " meters")
                self.total_distance = 0.0



if __name__ == "__main__":
    rospy.init_node('waypoint_control_node', log_level=rospy.INFO) 
    wpc = WaypointControl()
    try:
        while not rospy.is_shutdown():
            wpc.begin()
    except rospy.ROSInterruptException:
        pass