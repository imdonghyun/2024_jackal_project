#!/usr/bin/env python3
import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import tf
from conf import *
# position remember

HOME = 0
NEXT_GOAL = 1
TRASHCAN = 2


# home = [0,0,0]
# goal1 = [0,0,0]
# goal2 = [0,0,0]
# trash_can = [0,0,0]

pos_list = (home, goal1, goal2, trash_can)

next_goal_from_finder = 0

lab_center = [4.13, -1.8, 1.0]
#
##
pos1=home
robot_postion = [0,0,1.0]

def next_update(data):
    global next_goal_from_finder
    next_goal_from_finder = data
##
def go_to_target(TARGET):
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    rospy.logwarn(TARGET)
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    pos1 = [0,0,1.0]
    rospy.loginfo(TARGET)
    rospy.loginfo("Home = 0, Next = 1, Trashcan = 2")
    if TARGET.data == HOME:
        pos1 = home.copy()
    elif TARGET.data == NEXT_GOAL:
        goal_listener = rospy.Subscriber("finder", Int32, next_update, queue_size=1)
        rospy.loginfo(next_goal_from_finder)
        pos1 = pos_list[(next_goal_from_finder + 1) % 3]
        goal_listener.unregister()
        #next_act
    elif TARGET.data == TRASHCAN:
        pos1 = trash_can.copy()
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    # goal.target_pose.header.frame_id = "camera_color_frame"
    goal.target_pose.header.stamp = rospy.Time.now()
    rospy.logwarn(pos1)
    goal.target_pose.pose.position.x = pos1[0]
    goal.target_pose.pose.position.y = pos1[1]
    goal.target_pose.pose.orientation.w = pos1[2]
    
    
    
    a = client.get_state()
    rospy.loginfo(a)
    client.send_goal(goal)

    a = client.get_state()
    rospy.loginfo(a)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
    
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        rospy.loginfo(client.get_result())

# print now position in map

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



# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        goal_listener = rospy.Subscriber("next_act", Int32, go_to_target)

        rospy.spin()
        ### pos read ###
        # listener = tf.TransformListener()
        # r = rospy.Rate(1)
        # robot_pos = get_robot_postion(listener,r)
        # if robot_pos != 1:
        #     rospy.loginfo(robot_pos)
        # else:
        #     rospy.logwarn("Cannot receiece tf data")
        ##############################


        ### set goal ###
        # result = movebase_client(lab_center)
        # print("result1 =", result)
        # result2 = movebase_client(home)
        # print("result2 =", result2)
        # if result:
        #     rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
