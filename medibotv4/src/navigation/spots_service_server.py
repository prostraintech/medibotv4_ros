#! /usr/bin/env python
import numpy
import rospy
import rospkg 
import time
import string
from medibotv4.srv import *
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal


class SaveSpots(object):
    def __init__(self):
        self.rospack_path = rospkg.RosPack().get_path('medibotv4')
        self.file_path = '/config/navigation/spots.yaml'
        self._pose = PoseWithCovarianceStamped()
        self.detection_dict = {}
        self.readFile()
        self.writeFile()
        _ = rospy.Service('/spots/set_spot', SetSpot , self.set_spot_srv_callback)
        _ = rospy.Service('/spots/send_goal', SendGoal , self.send_goal_srv_callback)
        _ = rospy.Service('/spots/get_spot', GetSpot, self.get_spot_srv_callback) 

        self._pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped , self.sub_callback)      
        self._goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=10)
        
        
    def reindent(self, s):
        s = string.split(s, '\n')
        s = [('  ') + line for line in s]
        s = string.join(s, '\n')
        return s

    def writeFile(self):
        with open(self.rospack_path + self.file_path, 'w') as outfile:
            if bool(self.detection_dict) == True:
                for i, (key, value) in enumerate(self.detection_dict.items()):
                    if value:
                        outfile.write(str(key) + ':\n' + self.reindent(str(value.pose.pose)) + ('\n\n' if (i+1!=len(self.detection_dict)) else ''))
            else:
                outfile.write('')


    def readFile(self):
        with open(self.rospack_path + self.file_path, 'r') as infile:
            s = infile.read() 
        s = string.split(s,'\n')
        for i,j in enumerate(s):
            s[i] = s[i].strip()
            s[i] = s[i].strip(':')
        inpose = PoseWithCovarianceStamped()
        for i,j in enumerate(s):
            try:
                if j[0]=='x' and s[i-1]=='position':
                    inpose = PoseWithCovarianceStamped()
                    inpose.pose.pose.position.x = float(j[3:])
                elif j[0]=='y' and s[i-2]=='position':
                    inpose.pose.pose.position.y = float(j[3:])
                elif j[0]=='z' and s[i-3]=='position':
                    inpose.pose.pose.position.z = float(j[3:])
                elif j[0]=='x' and s[i-1]=='orientation':
                    inpose.pose.pose.orientation.x = float(j[3:])
                elif j[0]=='y' and s[i-2]=='orientation':
                    inpose.pose.pose.orientation.y = float(j[3:])
                elif j[0]=='z' and s[i-3]=='orientation':
                    inpose.pose.pose.orientation.z = float(j[3:])
                elif j[0]=='w':
                    inpose.pose.pose.orientation.w = float(j[3:])
                    self.detection_dict[s[i-9]] = inpose
            except:
                pass


    def sub_callback(self, msg):
        self._pose = msg
    
    def set_spot_srv_callback(self, request):    
        label = request.label
        action = request.action
        response = SetSpotResponse()
        if action == "add":
            self.detection_dict[label] = self._pose
            self.writeFile()
            response.message = "Added Pose for " + label   
        elif action == "remove":
            if label in self.detection_dict: del self.detection_dict[label]
            self.writeFile()
            response.message = "Deleted Pose for " + label  
        elif action == "clear":
            self.detection_dict.clear()
            self.writeFile()
            response.message = "Cleared all pose"

        response.success = True
        return response

    def get_spot_srv_callback(self, request):    
        response = GetSpotResponse()
        for key, value in self.detection_dict.items():
            response.label.append(key)
        response.success = True
        response.message = "Returned spots label"
        return response



    def send_goal_srv_callback(self, request):
        label = request.label
        response = SendGoalResponse()
        Goal = MoveBaseActionGoal()
        Goal.goal.target_pose.header.frame_id='map'
        Goal.goal.target_pose.pose=self.detection_dict[label].pose.pose
        self._goal_pub.publish(Goal)
        response.message = "Goal to "+label+" is sent!"
        response.success = True
        return response

if __name__ == "__main__":
    rospy.init_node('spots_service_server', log_level=rospy.INFO) 
    spots_object = SaveSpots()
    rospy.spin() # mantain the service open.