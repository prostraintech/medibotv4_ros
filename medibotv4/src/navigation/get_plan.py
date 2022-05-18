#! /usr/bin/env python
import rospy
import math
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
from move_base_msgs.msg import MoveBaseActionGoal


class GetPath(object):
    def __init__(self):
        _ = rospy.Service('/medibot/get_plan', Trigger , self.get_plan_srv_callback)
        self.robot1_start = PoseWithCovarianceStamped()
        self.robot2_start = PoseWithCovarianceStamped()
        self.robot_goal = PoseStamped()

        self.robot1_start_sub = rospy.Subscriber('/robot_1/amcl_pose', PoseWithCovarianceStamped, self.robot1_start_sub_callback)
        self.robot2_start_sub = rospy.Subscriber('/robot_2/amcl_pose', PoseWithCovarianceStamped, self.robot2_start_sub_callback)
        self.goal_sub = rospy.Subscriber('/rviz_goal', PoseStamped, self.goal_sub_callback)
        self.robot1_goal_pub = rospy.Publisher("/robot_1/move_base/goal", MoveBaseActionGoal, queue_size=10)
        self.robot2_goal__pub = rospy.Publisher("/robot_2/move_base/goal", MoveBaseActionGoal, queue_size=10)
        


    def sub_callback(self, msg):
        self._pose = msg
    
    def robot1_start_sub_callback(self, msg):
        self.robot1_start = msg
    

    def robot2_start_sub_callback(self, msg):
        self.robot2_start = msg

    
    def goal_sub_callback(self, msg):
        self.robot_goal = msg
        self.robot_goal = msg


    def get_plan_srv_callback(self, request):    
        
        response = TriggerResponse()
        rospy.wait_for_service('/robot_1/move_base/make_plan')
        make_plan_service = rospy.ServiceProxy('/robot_1/move_base/make_plan', GetPlan)
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
        rospy.loginfo("robot_1 Total Distance = "+str(total_distance_1)+" meters")

        rospy.wait_for_service('/robot_2/move_base/make_plan')
        make_plan_service = rospy.ServiceProxy('/robot_2/move_base/make_plan', GetPlan)
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
        rospy.loginfo("robot_2 Total Distance = "+str(total_distance_2)+" meters")
        Goal = MoveBaseActionGoal()
        Goal.goal.target_pose.header.frame_id='map'
        Goal.goal.target_pose.pose = self.robot_goal.pose

        if total_distance_1 > total_distance_2:
            response.message = "robot_2 is nearer... assign task to robot_2"
            self.robot2_goal__pub.publish(Goal)
        else:
            response.message = "robot_1 is nearer... assign task to robot_1"
            self.robot1_goal__pub.publish(Goal)

        response.success = True
        return response

   
if __name__ == "__main__":
    rospy.init_node('Get_plan_service_server', log_level=rospy.INFO) 
    spots_object = GetPath()
    rospy.spin() # mantain the service open.
