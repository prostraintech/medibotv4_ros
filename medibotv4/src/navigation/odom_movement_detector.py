#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class MovementDetector(object):
    def __init__(self):
        """Initialize an object of the MovementDetector class."""
        # _mved_distance is for stored distance moved
        # create and initialize it here. Initial value is 0.0
        self._mved_distance = Float64()
        self._mved_distance.data = 0.0

        # Get the inital position. This will be a reference point for calculating
        # the distance moved 
        self.get_init_position()

        # Create a publisher for publishing the distance moved into the topic '/moved_distance'
        self.distance_moved_pub = rospy.Publisher('/moved_distance', Float64, queue_size=1)

        # create a subscriber for getting new Odometry messages
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def get_init_position(self):
        """Get the initial position of the robot."""
        """
        Structure of the odom position message:
        user:~$ rostopic echo /odom -n1
        header:
        seq: 14929
        stamp:
            secs: 748
            nsecs: 215000000
        frame_id: "odom"
        child_frame_id: "base_footprint"
        pose:
        pose:
            position:
            x: 0.00668370211388
            y: 0.00010960687178
            z: -0.000246865753431
        """
        data_odom = None
        # wait for a message from the odometry topic and store it in data_odom when available
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message("/odom", Odometry, timeout=1)
            except:
                rospy.loginfo("Current odom not ready yet, retrying for setting up init pose")
        
        # Store the received odometry "position" variable in a Point instance 
        self._current_position = Point()
        self._current_position.x = data_odom.pose.pose.position.x
        self._current_position.y = data_odom.pose.pose.position.y
        self._current_position.z = data_odom.pose.pose.position.z

    def odom_callback(self, msg):
        """Process odometry data sent by the subscriber."""
        # Get the position information from the odom message
        # See the structure of an /odom message in the `get_init_position` function
        NewPosition = msg.pose.pose.position

        # Calculate the new distance moved, and add it to _mved_distance and 
        self._mved_distance.data += self.calculate_distance(NewPosition, self._current_position)
        
        # Update the current position of the robot so we have a new reference point
        # (The robot has moved and so we need a new reference for calculations)
        self.updatecurrent_positin(NewPosition)
        
        # If distance moved is big enough, publish it to the designated topic
        # Otherwise publish zero
        if self._mved_distance.data < 0.000001:
            aux = Float64()
            aux.data = 0.0
            self.distance_moved_pub.publish(aux)
        else:
            self.distance_moved_pub.publish(self._mved_distance)

    def updatecurrent_positin(self, new_position):
        """Update the current position of the robot."""
        self._current_position.x = new_position.x
        self._current_position.y = new_position.y
        self._current_position.z = new_position.z

    def calculate_distance(self, new_position, old_position):
        """Calculate the distance between two Points (positions)."""
        x2 = new_position.x
        x1 = old_position.x
        y2 = new_position.y
        y1 = old_position.y
        dist = math.hypot(x2 - x1, y2 - y1)
        return dist

    def publish_moved_distance(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    # create a node for running the program
    rospy.init_node('movement_detector_node', anonymous=True)

    # create an instance of the MovementDetector class and set the code
    # in motion
    movement_obj = MovementDetector()
    movement_obj.publish_moved_distance()