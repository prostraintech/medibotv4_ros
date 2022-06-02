#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

def my_callback(msg):
    """Callback function that processes messages from the subscriber."""

    # get the distance moved from the message
    distance_moved = msg.data

    # If distance is less than 2, continue moving the robot
    # Otherwise, stop it (by pubishing `0`)
    if msg.data < 2:
        move.linear.x = 0.1

    if msg.data > 2:
        move.linear.x = 0

    pub.publish(move)

# create a node for running the program
rospy.init_node('test_movement')

# create a subscriber that gets the distance moved
sub = rospy.Subscriber('/moved_distance', Float64, my_callback)

# Create a publisher that moves the robot
pub = rospy.Publisher('/cmd_vel', Twist, queue_size="1")

# Create a global variable for publising a Twist ("cmd_vel") message 
move = Twist()

# Keep the program running
rospy.spin()