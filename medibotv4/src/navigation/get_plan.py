#! /usr/bin/env python


import math
import rospy
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
import sys 

amcl_pose = PoseWithCovarianceStamped()
goal_pose = PoseStamped()

def start_sub_callback(msg):
    global amcl_pose
    amcl_pose = msg

def goal_sub_callback(msg):
    global goal_pose
    goal_pose = msg

rospy.init_node('service_client')
rospy.wait_for_service('/robot_1/move_base/make_plan')
make_plan_service = rospy.ServiceProxy('/robot_1/move_base/make_plan', GetPlan)
start_sub = rospy.Subscriber('/robot_1/amcl_pose', PoseWithCovarianceStamped, start_sub_callback)
goal_sub = rospy.Subscriber('/robot_1/move_base_simple/goal', PoseStamped, goal_sub_callback)
msg = GetPlanRequest()
msg.start.header.frame_id = 'map'
msg.start.pose.position.x = amcl_pose.pose.pose.position.x
msg.start.pose.position.y = amcl_pose.pose.pose.position.y
msg.start.pose.position.z = amcl_pose.pose.pose.position.z
msg.start.pose.orientation.x = amcl_pose.pose.pose.orientation.x
msg.start.pose.orientation.y = amcl_pose.pose.pose.orientation.y
msg.start.pose.orientation.z = amcl_pose.pose.pose.orientation.z
msg.start.pose.orientation.w = amcl_pose.pose.pose.orientation.w
msg.goal.header.frame_id = 'map'
msg.goal.pose.position.x = 3
msg.goal.pose.position.y = -9
msg.goal.pose.position.z = 0
msg.goal.pose.orientation.x = 0
msg.goal.pose.orientation.y = 0
msg.goal.pose.orientation.z = 0
msg.goal.pose.orientation.w = 0
result = make_plan_service(msg)
# print (result)
first_time = True
prev_x = 0.0
prev_y = 0.0
total_distance_1 = 0.0
if len(result.plan.poses) > 0:
    for current_point in result.plan.poses:
        x = current_point.pose.position.x
        y = current_point.pose.position.y
        if not first_time:
            total_distance_1 += math.hypot(prev_x - x, prev_y - y) 
        else:
            first_time = False
        prev_x = x
        prev_y = y
    print ("robot_2 Total Distance = "+str(total_distance_1)+" meters")

rospy.init_node('service_client')
rospy.wait_for_service('/robot_2/move_base/make_plan')
make_plan_service = rospy.ServiceProxy('/robot_2/move_base/make_plan', GetPlan)
start_sub = rospy.Subscriber('/robot_2/amcl_pose', PoseWithCovarianceStamped, start_sub_callback)
goal_sub = rospy.Subscriber('/robot_1/move_base_simple/goal', PoseStamped, goal_sub_callback)
msg = GetPlanRequest()
msg.start.header.frame_id = 'map'
msg.start.pose.position.x = amcl_pose.pose.pose.position.x
msg.start.pose.position.y = amcl_pose.pose.pose.position.y
msg.start.pose.position.z = amcl_pose.pose.pose.position.z
msg.start.pose.orientation.x = amcl_pose.pose.pose.orientation.x
msg.start.pose.orientation.y = amcl_pose.pose.pose.orientation.y
msg.start.pose.orientation.z = amcl_pose.pose.pose.orientation.z
msg.start.pose.orientation.w = amcl_pose.pose.pose.orientation.w
msg.goal.header.frame_id = 'map'
msg.goal.pose.position.x = 3
msg.goal.pose.position.y = -9
msg.goal.pose.position.z = 0
msg.goal.pose.orientation.x = 0
msg.goal.pose.orientation.y = 0
msg.goal.pose.orientation.z = 0
msg.goal.pose.orientation.w = 0
result = make_plan_service(msg)
# print (result)
first_time = True
prev_x = 0.0
prev_y = 0.0
total_distance_2 = 0.0
if len(result.plan.poses) > 0:
    for current_point in result.plan.poses:
        x = current_point.pose.position.x
        y = current_point.pose.position.y
        if not first_time:
            total_distance_2 += math.hypot(prev_x - x, prev_y - y) 
        else:
            first_time = False
        prev_x = x
        prev_y = y
    print ("robot_2 Total Distance = "+str(total_distance_2)+" meters")

if total_distance_1 > total_distance_2:
    print("robot_2 is nearer... assign task to robot_2")
else:
    print("robot_1 is nearer... assign task to robot_1")
