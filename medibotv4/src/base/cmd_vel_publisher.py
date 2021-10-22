#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('publisher_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = Twist()
    msg.linear.x=0
    msg.angular.z=0
    while not rospy.is_shutdown():
        rospy.loginfo("publishing to /cmd_vel, lin="+str(msg.linear.x)+" , ang="+str(msg.angular.z))
        pub.publish(msg)
        rate.sleep()
	msg.linear.x+=0.01
    	msg.angular.z+=0.01

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass