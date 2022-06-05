#! /usr/bin/env python
import math
import roslib, rospy
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback

class CalculateDistanceTraveled(object):
    def __init__(self, robot_namespace=""):
        self.total_distance = 0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.goal_point = []
        self.first_time = True
        if robot_namespace != "":
            robot_namespace = "/" + robot_namespace
        self.pose_subscriber = rospy.Subscriber(robot_namespace+"/move_base/feedback", MoveBaseActionFeedback, self.addPointToTotalDistance)
        self.goal_subscriber = rospy.Subscriber(robot_namespace+"/move_base/goal", MoveBaseActionGoal, self.getGoalPoint)

    def distancePoints(self,x1,y1,x2,y2):
        return math.hypot(x2 - x1, y2 - y1)

    def getGoalPoint(self,g):
        self.goal_point = g

    def addPointToTotalDistance(self,current_point):
        if self.first_time:
            self.first_time = False
            self.prev_x = current_point.feedback.base_position.pose.position.x
            self.prev_y = current_point.feedback.base_position.pose.position.y
        else:
            x = current_point.feedback.base_position.pose.position.x
            y = current_point.feedback.base_position.pose.position.y

            self.total_distance += self.distancePoints(x, y, self.prev_x, self.prev_y)

            self.prev_x = x
            self.prev_y = y
            #print("Current distance traveled= " + str(self.total_distance) + " meters")
            if self.goal_point != []:
                if self.distancePoints(x, y, self.goal_point.goal.target_pose.pose.position.x, self.goal_point.goal.target_pose.pose.position.y) <= 0.1:
                    self.pose_subscriber.unregister()
                    self.goal_subscriber.unregister()
                    #print("Total Distance = " + str(self.total_distance) + " meters")

    def getTotalDistance(self):
        return self.total_distance