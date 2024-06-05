#!/usr/bin/env python3
import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import tf, math
from conf import *


def get_robot_postion(listener,r):
    for _ in range(5):
        if listener.canTransform(target_frame="base_link", source_frame="map", time=rospy.Time(0)):
            tf_matrix = listener.lookupTransform(target_frame="base_link", source_frame="map", time=rospy.Time(0))
            # rospy.loginfo(tf_matrix)
            return (tf_matrix[0][0], tf_matrix[0][1], tf_matrix[1][3])
        # 변환 행렬 리턴. 리턴 값은 ([x, y, z], [x, y, z, w])
        else:
            r.sleep()          
    return 1

def get_check_point(robot_pos):
    global tt
    tt = 0
    home_dist = math.sqrt(robot_pos[0]**2 + robot_pos[1]**2)
    goal1_dist = math.sqrt((robot_pos[0]-goal1[0])**2 + (robot_pos[1]-goal1[1])**2)
    goal2_dist = math.sqrt((robot_pos[0]-goal2[0])**2 + (robot_pos[1]-goal2[1])**2)

    if home_dist < 0.5:
        tt = 0
        rospy.loginfo("near home")
    elif goal1_dist < 0.5:
        tt = 1
        rospy.loginfo("near goal1")
    elif goal2_dist < 0.5:
        tt = 2
        rospy.loginfo("near goal2")
    return tt
    
if __name__ =='__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('finder',  anonymous=True)
        finder_pub = rospy.Publisher("finder", Int32, queue_size=1)

        

        listener = tf.TransformListener()
        r = rospy.Rate(1)
        check_point = Int32()
        
        while not rospy.is_shutdown():
            # rospy.loginfo("on")
            robot_pos = get_robot_postion(listener,r)

            if robot_pos != 1:
                check_point.data = get_check_point(robot_pos)
                finder_pub.publish(check_point)
                rospy.loginfo(check_point)
            else:
                rospy.logwarn("Cannot receiece tf data")
            r.sleep()


        ### pos read ###
        # listener = tf.TransformListener()
        # r = rospy.Rate(1)
        # robot_pos = get_robot_postion(listener,r)
        # if robot_pos != 1:
        #     rospy.loginfo(robot_pos)
        # else:
        #     rospy.logwarn("Cannot receiece tf data")
        ##############################
    except rospy.ROSInterruptException:
        rospy.loginfo("Finder finished.")