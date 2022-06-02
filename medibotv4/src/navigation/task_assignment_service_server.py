#! /usr/bin/env python
import rospy
import math
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray


class TaskAssign(object):
    def __init__(self):
        _ = rospy.Service('/medibotv4/task_assign', Trigger , self.task_assign_srv_callback)
        self.robot1_start = PoseWithCovarianceStamped()
        self.robot2_start = PoseWithCovarianceStamped()
        self.robot_goal = PoseStamped()
        self.robot1_status = "idle"
        self.robot2_status = "idle"

        self.robot1_start_sub = rospy.Subscriber('/robot1/amcl_pose', PoseWithCovarianceStamped, self.robot1_start_sub_callback)
        self.robot2_start_sub = rospy.Subscriber('/robot2/amcl_pose', PoseWithCovarianceStamped, self.robot2_start_sub_callback)
        self.goal_sub = rospy.Subscriber('/nav_goal', PoseStamped, self.goal_sub_callback)
        self.robot1_goal_pub = rospy.Publisher("/robot1/move_base/goal", MoveBaseActionGoal, queue_size=10)
        self.robot2_goal_pub = rospy.Publisher("/robot2/move_base/goal", MoveBaseActionGoal, queue_size=10)
        self.robot1_status_sub = rospy.Subscriber("/robot1/move_base/status", GoalStatusArray, self.robot1_status_sub_callback)
        self.robot2_status_sub = rospy.Subscriber("/robot2/move_base/status", GoalStatusArray, self.robot2_status_sub_callback)
        


    def sub_callback(self, msg):
        self._pose = msg
    
    def robot1_start_sub_callback(self, msg):
        self.robot1_start = msg
    

    def robot2_start_sub_callback(self, msg):
        self.robot2_start = msg

    
    def goal_sub_callback(self, msg):
        self.robot_goal = msg

    def robot1_status_sub_callback(self,msg):
        i = 0
        while i<len(msg.status_list):
            if msg.status_list[i].status == 1:
                self.robot1_status = "active"
            else:
                self.robot1_status = "idle"
            i+=1

    def robot2_status_sub_callback(self,msg):
        i = 0
        while i<len(msg.status_list):
            if msg.status_list[i].status == 1:
                self.robot2_status = "active"
            else:
                self.robot2_status = "idle"
            i+=1


    def task_assign_srv_callback(self, request):
        response = TriggerResponse()    
        if self.robot1_status == "idle" and self.robot2_status == "idle":
            rospy.wait_for_service('/robot1/move_base/make_plan')
            make_plan_service = rospy.ServiceProxy('/robot1/move_base/make_plan', GetPlan)
            msg = GetPlanRequest()
            msg.start.header = self.robot1_start.header
            msg.start.pose = self.robot1_start.pose.pose
            msg.goal = self.robot_goal

            result = make_plan_service(msg)

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
            rospy.loginfo("robot1 Total Distance = "+str(total_distance_1)+" meters")

            rospy.wait_for_service('/robot2/move_base/make_plan')
            make_plan_service = rospy.ServiceProxy('/robot2/move_base/make_plan', GetPlan)
            msg = GetPlanRequest()
            msg.start.header = self.robot2_start.header
            msg.start.pose = self.robot2_start.pose.pose
            msg.goal = self.robot_goal
            result = make_plan_service(msg)

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
            rospy.loginfo("robot2 Total Distance = "+str(total_distance_2)+" meters")
            # Goal = MoveBaseActionGoal()
            # Goal.goal.target_pose.header.frame_id='map'
            # Goal.goal.target_pose.pose = self.robot_goal.pose

            if total_distance_1 > total_distance_2:
                response.message = "robot2"
                # self.robot2_goal_pub.publish(Goal)
            else:
                response.message = "robot1"
                # self.robot1_goal_pub.publish(Goal)

            response.success = True
            return response
        elif self.robot1_status == "idle" and self.robot2_status == "active":
            rospy.loginfo("robot2 is busy... assigning task to robot1")
            rospy.wait_for_service('/robot1/move_base/make_plan')
            make_plan_service = rospy.ServiceProxy('/robot1/move_base/make_plan', GetPlan)
            msg = GetPlanRequest()
            msg.start.header = self.robot1_start.header
            msg.start.pose = self.robot1_start.pose.pose
            msg.goal = self.robot_goal

            result = make_plan_service(msg)

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
            rospy.loginfo("robot1 Total Distance = "+str(total_distance_1)+" meters")
            response.message = "robot1"
            response.success = True
            return response
        elif self.robot1_status == "active" and self.robot2_status == "idle":
            rospy.loginfo("robot1 is busy... assigning task to robot2")
            rospy.wait_for_service('/robot2/move_base/make_plan')
            make_plan_service = rospy.ServiceProxy('/robot2/move_base/make_plan', GetPlan)
            msg = GetPlanRequest()
            msg.start.header = self.robot2_start.header
            msg.start.pose = self.robot2_start.pose.pose
            msg.goal = self.robot_goal
            result = make_plan_service(msg)

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
            rospy.loginfo("robot2 Total Distance = "+str(total_distance_2)+" meters")
            response.message = "robot2"
            response.success = True
            return response
        else:
            rospy.loginfo("both robots are busy... please wait for robot to complete task or cancel robot task manually")
            response.message = "none"
            response.success = True
            return response

   
if __name__ == "__main__":
    rospy.init_node('task_assignment_service_server', log_level=rospy.INFO) 
    task_object = TaskAssign()
    rospy.spin() # mantain the service open.
