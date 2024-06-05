#!/usr/bin/env python3
# license removed for brevity

import rospy
import time
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from my_pkg.msg import marker
from std_msgs.msg import Bool

stop_sign = False #False : need to stop True : no need to stop

trash_checker = 0

def stop_move_client():
   
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

    
    client.wait_for_server()
    # a = client.get_goal_status_text()
    # rospy.loginfo(a)

    # time.sleep(4)

    client.cancel_all_goals()
   # Waits for the server to finish performing the action.
    # wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    # if not wait:
    #     rospy.logerr("Action server not available!")
    #     rospy.signal_shutdown("Action server not available!")
    # else:
    # Result of executing the action
    return client.get_result()   

def check_marker(vector):
    global grip
    global stop_sign
    global trash_checker
    ids = vector.id
    if ids == 1:
        trash_checker = 0
    #if stop_sign == False:
    if vector.y < 1000:
        if vector.x < 250 and vector.x > -200:
            rospy.loginfo("Marker Detected!")
            try:
                if trash_checker == 0:
                  result = stop_move_client()
                  if ids == 0:
                      trash_checker = 1
                  if result:
                      rospy.loginfo("Goal execution done!")
                      
                      r.sleep()
                elif trash_checker == 1:
                  rospy.loginfo('No need to stop goal')
            #TODO stop goal and follow marker position
            except rospy.ROSInterruptException:
                rospy.loginfo("Navigation test finished.")
            # else:
            #     rospy.info("keep going")
    #else:
        #rospy.loginfo('No need to stop goal')

def stop_sign_switcher(data):
    global stop_sign
    rospy.loginfo(data)
    stop_sign = data

if __name__ == '__main__':
    rospy.init_node('stop_goal_py')
    r = rospy.Rate(1)
    marker_checker = rospy.Subscriber('marker_vec', marker, check_marker, queue_size=1)
    #grip_checker = rospy.Subscriber('stop_sign', Bool, stop_sign_switcher)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    

