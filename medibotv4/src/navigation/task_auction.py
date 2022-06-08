#! /usr/bin/env python
import rospy
import math
import time
from medibotv4.srv import *
from actionlib_msgs.msg import GoalStatusArray, GoalID
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from calculate_distance_traveled import CalculateDistanceTraveled


class TaskAuction(object):
    # def __init__(self, task_list=["taskA", "taskB","taskC", "taskD", "taskE", "taskF"]):
    def __init__(self, task_list=["task1", "task2", "task3", "task4"]):
        get_spot_client = rospy.ServiceProxy("/spots/get_spot", GetSpot)
        request = GetSpotRequest()
        response = get_spot_client(request)
        self.tasks = {}
        for i in range(0,len(response.label)):
            self.tasks[response.label[i]] = response.pose[i]
        # self.tasks['spotA'].pose =
        self.robot1_initial_position = "robot1_initial_pose" # simulation start position
        self.robot2_initial_position = "robot2_initial_pose"
        self.task_list = task_list
        self.robot1_task_list = []
        self.robot2_task_list = []
        self.robot1_start = PoseWithCovarianceStamped()
        self.robot2_start = PoseWithCovarianceStamped()
        self.robot_goal = PoseStamped()
        self.robot1_status = "idle"
        self.robot2_status = "idle"
        self.robot1_status_sub = rospy.Subscriber("/robot1/move_base/status", GoalStatusArray, self.robot1_status_sub_callback, queue_size=1)
        self.robot2_status_sub = rospy.Subscriber("/robot2/move_base/status", GoalStatusArray, self.robot2_status_sub_callback, queue_size=1)
        self.robot1_status_sub = rospy.Subscriber("/robot1/move_base/status", GoalStatusArray, self.robot1_status_sub_callback)
        self.robot2_status_sub = rospy.Subscriber("/robot2/move_base/status", GoalStatusArray, self.robot2_status_sub_callback)
        self.robot1_mb_cancel_pub = rospy.Publisher('/robot1/move_base/cancel', GoalID, queue_size=10)
        self.robot2_mb_cancel_pub = rospy.Publisher('/robot2/move_base/cancel', GoalID, queue_size=10)       
        self.Ntest = 1
        self.start_time = 0.0
        self.end_time = 0.0
        self.robot1_total_distance = 0.0
        self.robot2_total_distance = 0.0
        self.total_distance_travelled = 0.0
        self.ir1=0
        self.ir2=0
        self.idle_count1 = 0
        self.idle_count2 = 0
        rospy.on_shutdown(self.shutdownhook)
        rospy.wait_for_service('/spots/send_goal')

    def robot1_start_sub_callback(self, msg):
        self.robot1_start = msg
    
    def robot2_start_sub_callback(self, msg):
        self.robot2_start = msg

    def robot1_status_sub_callback(self,msg):
        for i in range(0, len(msg.status_list)):
            if len(msg.status_list) == 3:
                if msg.status_list[0].status == 1 or msg.status_list[1].status == 1 or msg.status_list[2].status == 1:
                    self.robot1_status = "active"
                    self.idle_count1 = 0
                    break
                else:
                    self.idle_count1 += 1
                    if self.idle_count1 > 3:
                        self.robot1_status = "idle"
            if len(msg.status_list) == 2:
                if msg.status_list[0].status == 1 or msg.status_list[1].status == 1:
                    self.robot1_status = "active"
                    self.idle_count1 = 0
                    break
                else:
                    self.idle_count1 += 1
                    if self.idle_count1 > 3:
                        self.robot1_status = "idle"
            if len(msg.status_list) == 1:
                if msg.status_list[0].status == 1:
                    self.robot1_status = "active"
                    self.idle_count1 = 0
                    break
                if msg.status_list[0].status == 3:
                    self.idle_count1 += 1
                    if self.idle_count1 > 3:
                        self.robot1_status = "idle"
            else:
                self.idle_count1 += 1
                if self.idle_count1 > 3:
                    self.robot1_status = "idle"
        

    def robot2_status_sub_callback(self,msg):
        for i in range(0, len(msg.status_list)):
            if len(msg.status_list) == 3:
                if msg.status_list[0].status == 1 or msg.status_list[1].status == 1 or msg.status_list[2].status == 1:
                    self.robot2_status = "active"
                    self.idle_count2 = 0
                    break
                else:
                    self.idle_count2 += 1
                    if self.idle_count2 > 3:
                        self.robot2_status = "idle"
            if len(msg.status_list) == 2:
                if msg.status_list[0].status == 1 or msg.status_list[1].status == 1:
                    self.robot2_status = "active"
                    self.idle_count2 = 0
                    break
                else:
                    self.idle_count2 += 1
                    if self.idle_count2 > 3:
                        self.robot2_status = "idle"
            if len(msg.status_list) == 1:
                if msg.status_list[0].status == 1:
                    self.robot2_status = "active"
                    self.idle_count2 = 0
                    break
                if msg.status_list[0].status == 3:
                    self.idle_count2 += 1
                    if self.idle_count2 > 3:
                        self.robot2_status = "idle"
            else:
                self.idle_count2 += 1
                if self.idle_count2 > 3:
                    self.robot2_status = "idle"

    def shutdownhook(self):
        msg = GoalID()
        self.robot1_mb_cancel_pub.publish(msg)
        self.robot2_mb_cancel_pub.publish(msg)

    def ready_initial_position(self):
        print("robot moving to initial position")
        robot_send_goal_client = rospy.ServiceProxy("/spots/send_goal", SendGoal)
        request = SendGoalRequest()
        request.label = self.robot1_initial_position
        request.ns = "/robot1"
        response = robot_send_goal_client(request)
        time.sleep(2)
        robot_send_goal_client = rospy.ServiceProxy("/spots/send_goal", SendGoal)
        request = SendGoalRequest()
        request.label = self.robot2_initial_position
        request.ns = "/robot2"
        response = robot_send_goal_client(request)
        time.sleep(2)
        while self.robot1_status != "idle" and self.robot2_status != "idle":
            pass
        time.sleep(5)
        while self.robot1_status != "idle" and self.robot2_status != "idle":
            pass

    def begin(self):
        self.ready_initial_position()
        print("robot at initial position")

        for i in range(0, len(self.task_list)):
            rospy.wait_for_service('/robot1/move_base/make_plan')
            make_plan_service = rospy.ServiceProxy('/robot1/move_base/make_plan', GetPlan)
            msg = GetPlanRequest()
            msg.start.header = self.robot1_start.header
            msg.start.pose = self.robot1_start.pose.pose
            self.robot_goal.header.frame_id = 'map'
            self.robot_goal.pose = self.tasks[self.task_list[i]].pose.pose
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
                        total_distance_1 += math.hypot(prev_x - x, prev_y - y) # calculate robot1 planned path distance
                    else:
                        first_time = False
                    prev_x = x
                    prev_y = y
            print("robot1 path distance to "+self.task_list[i]+" is "+str(total_distance_1))

            rospy.wait_for_service('/robot2/move_base/make_plan')
            make_plan_service = rospy.ServiceProxy('/robot2/move_base/make_plan', GetPlan)
            msg = GetPlanRequest()
            msg.start.header = self.robot2_start.header
            msg.start.pose = self.robot2_start.pose.pose
            self.robot_goal.header.frame_id = 'map'
            self.robot_goal.pose = self.tasks[self.task_list[i]].pose.pose
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
                        total_distance_2 += math.hypot(prev_x - x, prev_y - y) # calculate robot2 planned path distance
                    else:
                        first_time = False
                    prev_x = x
                    prev_y = y
            print("robot2 path distance to "+self.task_list[i]+" is "+str(total_distance_2))

            if total_distance_1 < total_distance_2:
                self.robot1_task_list.insert(self.ir1, self.task_list[i])
                print("===> "+self.task_list[i]+' is assigned to robot1')
                self.ir1 += 1
            else:
                self.robot2_task_list.insert(self.ir2, self.task_list[i])
                print("===> "+self.task_list[i]+' is assigned to robot2')
                self.ir2 += 1

        for i in range(0, len(self.robot1_task_list)): # move robot1 to assigned tasks
            cdt1 = CalculateDistanceTraveled(robot_namespace="robot1") # start calculating distance travelled
            send_goal_service_client = rospy.ServiceProxy("/spots/send_goal", SendGoal)
            request = SendGoalRequest()
            request.label = self.robot1_task_list[i]
            request.ns = "/robot1"
            # check if assigning first task
            if i == 0:
                print("\n---------- TEST " + str(self.Ntest) + " ----------")
                self.start_time = rospy.get_time() # start the timer
            response = send_goal_service_client(request) # call service to send goal to robot1
            print(self.robot1_task_list[i]+":")
            print(" robot1 is moving to " + self.robot1_task_list[i])
            # wait until the goal is reached
            time.sleep(5)
            while self.robot1_status != "idle":
                pass
            # reached at spot
            self.robot1_total_distance += cdt1.getTotalDistance()

        for i in range(0, len(self.robot2_task_list)): # move robot2 to assigned tasks
            cdt2 = CalculateDistanceTraveled(robot_namespace="robot2") # start calculating distance travelled
            send_goal_service_client = rospy.ServiceProxy("/spots/send_goal", SendGoal)
            request = SendGoalRequest()
            request.label = self.robot2_task_list[i]
            request.ns = "/robot2"
            response = send_goal_service_client(request) # call service to send goal to robot1
            print(self.robot2_task_list[i]+":")
            print(" robot2 is moving to " + self.robot2_task_list[i])
            # wait until the goal is reached
            time.sleep(5)
            while self.robot2_status != "idle":
                pass
            # reached at spot
            self.robot2_total_distance += cdt2.getTotalDistance()
        
        # print experimental data and reset them for next test
        self.Ntest += 1
        self.end_time = rospy.get_time() # end the timer
        self.total_distance_travelled = self.robot1_total_distance + self.robot2_total_distance
        print(" Time taken = " + str(round(self.end_time - self.start_time,2)) + "seconds")
        print(" Robot1 total distance = " + str(self.robot1_total_distance) + "meters")
        print(" Robot2 total distance = " + str(self.robot2_total_distance) + "meters")
        print(" Total distance travelled = " + str(self.total_distance_travelled) + "meters")
        print("--------------------\n")
        self.start_time = 0.0
        self.end_time = 0.0
        self.robot1_total_distance = 0.0
        self.robot2_total_distance = 0.0
        self.total_distance_travelled = 0.0
        self.ir1 = 0
        self.ir2 = 0
        self.robot1_task_list = []
        self.robot2_task_list = []


if __name__ == "__main__":
    rospy.init_node('task_auction_node', log_level=rospy.INFO) 
    ta = TaskAuction()
    try:
        while not rospy.is_shutdown():
            ta.begin()
    except rospy.ROSInterruptException:
        pass


