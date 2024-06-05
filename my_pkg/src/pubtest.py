#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String, Bool, Int32

if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('chatter', Bool, queue_size=1)
    while 1:
        pub.publish(True)
        time.sleep(1)