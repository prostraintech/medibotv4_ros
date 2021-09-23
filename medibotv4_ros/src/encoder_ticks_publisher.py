#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

def talker():
    publ = rospy.Publisher('lwheel', Int16, queue_size=10)
    pubr = rospy.Publisher('rwheel', Int16, queue_size=10)
    rospy.init_node('encoder_publisher_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msgl = Int16()
    msgr = Int16()
    msgl.data=0
    msgr.data=0
    while not rospy.is_shutdown():
        rospy.loginfo("publishing to /lwheel="+str(msgl.data)+" , /rwheel="+str(msgr.data))
        publ.publish(msgl)
        pubr.publish(msgr)
        rate.sleep()
    	msgl.data+=1
        msgr.data+=1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass