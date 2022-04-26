#! /usr/bin/env python

import rospy
import rospkg 
import time
from medibotv4.srv import SaveSpotService, SaveSpotServiceResponse
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped


class SaveSpots(object):
    def __init__(self, srv_name='/save_spot'):
        self.rospack_path = rospkg.RosPack().get_path('medibotv4')
        self._srv_name = srv_name
        self._pose = PoseWithCovarianceStamped()
        self.detection_dict = {"spot1":self._pose, "spot2":self._pose, "spot3":self._pose}
        self._my_service = rospy.Service(self._srv_name, SaveSpotService , self.srv_callback)
        self._pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped , self.sub_callback)

    def sub_callback(self, msg):
        self._pose = msg
    
    def srv_callback(self, request):
        
        label = request.label
        response = SaveSpotServiceResponse()
        
        if label == "spot1":
            self.detection_dict["spot1"] = self._pose
            response.message = "Saved Pose for spot1 spot"
            
        elif label == "spot2":
            self.detection_dict["spot2"] = self._pose
            response.message = "Saved Pose for spot2 spot"
            
        elif label == "spot3":
            self.detection_dict["spot3"] = self._pose
            response.message = "Saved Pose for spot3 spot"
            
        elif label == "end":
            with open(self.rospack_path+'/spots/spots.txt', 'w') as file:
                
                for key, value in self.detection_dict.items():
                    if value:
                        file.write(str(key) + ':\n----------\n' + str(value) + '\n===========\n')
                
                response.message = "Written Poses to spots.txt file"
                
        else:
            response.message = "No label with this name. Try with spotx or end(to write the file)"
        
        
        response.success = True
        
        return response


if __name__ == "__main__":
    rospy.init_node('save_spots_service_server', log_level=rospy.INFO) 
    save_spots_object = SaveSpots()
    rospy.spin() # mantain the service open.