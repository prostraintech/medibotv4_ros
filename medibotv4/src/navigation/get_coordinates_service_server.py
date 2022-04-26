#! /usr/bin/env python

import rospy
import rospkg
from medibotv4.srv import SaveSpotService, SaveSpotServiceResponse
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from send_coordinates_action_client import SendCoordinates
import actionlib
import time
import os
import rosparam


class GetCoordinates(object):
    def __init__(self, srv_name='/get_coordinates'):
        self.rospack_path = rospkg.RosPack().get_path('medibotv4')
        self._srv_name = srv_name
        self._my_service = rospy.Service(self._srv_name, SaveSpotService , self.srv_callback)

    
    def srv_callback(self, request):
        
        label = request.label
        response = SaveSpotServiceResponse()

        os.chdir(self.rospack_path+"/spots")
        paramlist=rosparam.load_file("spots.yaml",default_namespace=None)
        
        for params,ns in paramlist: #ns,param
        
            for key, value in params.items():
                if key == request.label:
                    rosparam.upload_params(ns,params) #ns,param
                    response.message = "Correctly uploaded parameters"
                    
        send_coordinates = SendCoordinates(request.label)
        
        response.success = True
        
        return response


if __name__ == "__main__":
    rospy.init_node('get_coordinates_service_server', log_level=rospy.INFO) 
    get_coordinates_object = GetCoordinates()
    rospy.spin() # mantain the service open.