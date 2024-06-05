#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool

def callback(data):
    rospy.loginfo(data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', Bool, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
