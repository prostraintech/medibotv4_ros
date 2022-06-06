#! /usr/bin/env python
import rospy
import time
from medibotv4.srv import *
from actionlib_msgs.msg import GoalStatusArray, GoalID
from calculate_distance_traveled import CalculateDistanceTraveled


class RoundRobin(object):
    def __init__(self, tasks=["spotA", "spotB","spotC", "spotD"]): # actual tasks
    # def __init__(self, tasks=["task1", "task2","task3", "task4"]): # simulation tasks
        self.tasks = tasks
        self.robot1_tasks= []
        self.robot2_tasks= []
        self.robot1_initial_position = "robot1_start_pose" # actual robot start pose
        self.robot2_initial_position = "robot2_start_pose"
        # self.robot1_initial_position = "robot1_initial_pose" # simulation robot start pose
        # self.robot2_initial_position = "robot2_initial_pose"
        self.tasks_status = [] # unassigned/assigned/completed
        for i in range(0, len(self.tasks)):
            self.tasks_status.append("unassigned")
        self.robot1_status = "idle"
        self.robot2_status = "idle"
        self.robot1_status_sub = rospy.Subscriber("/robot1/move_base/status", GoalStatusArray, self.robot1_status_sub_callback, queue_size=1)
        self.robot2_status_sub = rospy.Subscriber("/robot2/move_base/status", GoalStatusArray, self.robot2_status_sub_callback, queue_size=1)
        self.robot1_mb_cancel_pub = rospy.Publisher('/robot1/move_base/cancel', GoalID, queue_size=10)
        self.robot2_mb_cancel_pub = rospy.Publisher('/robot2/move_base/cancel', GoalID, queue_size=10)
        self.Ntest = 1
        self.start_time = 0.0
        self.end_time = 0.0
        self.robot1_total_distance = 0.0
        self.robot2_total_distance = 0.0
        self.total_distance_travelled = 0.0
        self.idle_count1 = 0
        self.idle_count2 = 0
        rospy.on_shutdown(self.shutdownhook)
        rospy.wait_for_service('/spots/send_goal') # wait for send goal service 

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
    
    def begin(self):
        self.ready_initial_position()
        print("robot at initial position")
        for i in range(0, len(self.tasks)):
            if (i % 2) != 0: #robot2
                cdt2 = CalculateDistanceTraveled(robot_namespace="robot2") # start calculating distance travelled
                send_goal_service_client = rospy.ServiceProxy("/spots/send_goal", SendGoal)
                request = SendGoalRequest()
                request.label = self.tasks[i]
                request.ns = "/robot2"
                response = send_goal_service_client(request) # call service to send goal to robot1
                print("task"+str(i+1)+":")
                self.tasks_status[i] = "assigned"
                print(" "+self.tasks_status[i]+" task"+str(i+1)+" to robot2")
                print(" robot2 is moving to " + self.tasks[i])
                # wait until the goal is reached
                time.sleep(5)
                while self.robot2_status != "idle":
                    pass
                # reached at spot
                self.tasks_status[i] = "completed"
                print(" Task "+str(i+1)+" "+self.tasks_status[i])
                self.robot2_total_distance += cdt2.getTotalDistance()
                time.sleep(2)

            else: # robot1
                cdt1 = CalculateDistanceTraveled(robot_namespace="robot1") # start calculating distance travelled
                send_goal_service_client = rospy.ServiceProxy("/spots/send_goal", SendGoal)
                request = SendGoalRequest()
                request.label = self.tasks[i]
                request.ns = "/robot1"
                # check if assigning first task
                if i == 0:
                    print("\n---------- TEST " + str(self.Ntest) + " ----------")
                    self.start_time = rospy.get_time() # start the timer
                response = send_goal_service_client(request) # call service to send goal to robot1
                print("task"+str(i+1)+":")
                self.tasks_status[i] = "assigned"
                print(" "+self.tasks_status[i]+" task"+str(i+1)+" to robot1")
                print(" robot1 is moving to " + self.tasks[i])
                # wait until the goal is reached
                time.sleep(5)
                while self.robot1_status != "idle":
                    pass
                # reached at spot
                self.tasks_status[i] = "completed"
                print(" Task "+str(i+1)+" "+self.tasks_status[i])
                self.robot1_total_distance += cdt1.getTotalDistance()
                time.sleep(2)
                
            # check if final task is completed
            if self.tasks_status[-1] == "completed":
                self.Ntest += 1
                self.end_time = rospy.get_time() # end the timer
                self.total_distance_travelled = self.robot1_total_distance + self.robot2_total_distance
                print(" Time taken = " + str(round(self.end_time - self.start_time,2)) + "seconds")
                print(" Robot1 total distance = " + str(self.robot1_total_distance) + "meters")
                print(" Robot2 total distance = " + str(self.robot2_total_distance) + "meters")
                print(" Total distance travelled = " + str(self.total_distance_travelled) + "meters")
                print("\n--------------------\n")
                self.start_time = 0.0
                self.end_time = 0.0
                self.robot1_total_distance = 0.0
                self.robot2_total_distance = 0.0
                self.total_distance_travelled = 0.0
                for i in range(0, len(self.tasks)):
                    self.tasks_status[i] = "unassigned"


if __name__ == "__main__":
    rospy.init_node('round_robin_node', log_level=rospy.INFO) 
    rr = RoundRobin()
    try:
        while not rospy.is_shutdown():
            rr.begin()
    except rospy.ROSInterruptException:
        pass


