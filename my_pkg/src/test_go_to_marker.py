#!/usr/bin/env python3

import rospy

import numpy as np
import time, math
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool
from my_pkg.msg import marker
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

global velocity_avg_list
velocity_avg_list = []

class marker_follower:
    def __init__(self):
        self.marker_vector = "marker_vec"
        self.marker_reader = rospy.Subscriber(self.marker_vector, marker, self.marker_callback, queue_size=1)
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.robotarm_pub = rospy.Publisher('next_act', Int32, queue_size=1)
        self.grip = rospy.Publisher('grip', Bool, queue_size=1)
        #self.stop_sign = rospy.Publisher('stop_sign', Bool, queue_size=1)
        self.is_catch = False
        self.arrive = 0

    def valid_state(self): #true when 물건 잡으러 갈때, 물건 버리러 갈때
        if (not self.is_catch and self.marker_id==1) or (self.is_catch and self.marker_id==0):
            return True
        else:
            return False

    def grip_req(self):
        result = call_robotarm_service()

        if result == True:
            rospy.loginfo("Arm catched the material!!")
            self.is_catch = True
            self.robotarm_pub.publish(2)
            #TODO publish something
        else:
            rospy.logerr("Arm failed to catch it!")
            self.is_catch = False
            self.robotarm_pub.publish(1)
            #TODO publish error
        #self.stop_sign.publish(True)

    def put_req(self):
        result = call_robotarm_service()
        self.is_catch = False
        rospy.loginfo("successed in throwing away trash")
        #self.stop_sign.publish(True)
        self.robotarm_pub.publish(1)
        
    
    def arrive_checker(self, velocity):
        global velocity_avg_list
        num = 50
        
        present_vel = np.array([velocity.linear.x, velocity.linear.y, velocity.linear.z,
                                velocity.angular.x, velocity.angular.y, velocity.angular.z])
        vel_norm = np.linalg.norm(present_vel)

        velocity_avg_list.append(vel_norm)

        if (len(velocity_avg_list)>num):
            del velocity_avg_list[0]

        if sum(velocity_avg_list)==0:
            rospy.loginfo("jackal arrives!")
            return 1
        else:
            return 0
            
                #TODO check is still there material in the camera

    def marker_callback(self, vector):
        velocity = Twist()
        self.marker_id = vector.id
        self.x_margin = 5 # allowed x direction error, should be positive
        self.y_distance = 170 # y distance
        self.y_error = 5
        rospy.loginfo(self.marker_id)
        self.x_margin_trashcan = 5
        self.y_distance_trashcan = 250 # y distance
        self.y_error_trashcan = 10

        if not self.isInRange(vector.x, vector.y) and self.valid_state():
            #self.stop_sign.publish(False)

            if self.marker_id == 1:
                if vector.x < -self.x_margin:
                    velocity.angular.z = 0.1
                elif vector.x > self.x_margin:
                    velocity.angular.z = -0.1
                else:
                    velocity.angular.z = 0.0

                if vector.y > self.y_distance+self.y_error:
                    velocity.linear.x = 0.05
                elif vector.y < self.y_distance-self.y_error:
                    velocity.linear.x = -0.05
                else:
                    velocity.linear.x = 0.0
            elif self.marker_id == 0:
                if vector.x < -self.x_margin_trashcan:
                    velocity.angular.z = 0.1
                elif vector.x > self.x_margin_trashcan:
                    velocity.angular.z = -0.1
                else:
                    velocity.angular.z = 0.0

                if vector.y > self.y_distance_trashcan+self.y_error:
                    velocity.linear.x = 0.05
                elif vector.y < self.y_distance_trashcan-self.y_error:
                    velocity.linear.x = -0.05
                else:
                    velocity.linear.x = 0.0
            
            self.velocity_pub.publish(velocity)

        else:
            velocity.linear.x = 0
            velocity.angular.z = 0
            self.velocity_pub.publish(velocity)
            rospy.loginfo('calling arrived checker')
        
        arrive = self.arrive_checker(velocity)
        if arrive:

            if not self.is_catch and self.marker_id==1:
                self.grip_req()

            elif self.is_catch and self.marker_id==0:
                self.put_req()
            
            
            

    def isInRange(self, x, y):
        if self.marker_id == 1: # 1 = trash
            if abs(x) < self.x_margin and abs(y-self.y_distance) < self.y_error:
                return 1
            else:
                return 0
        elif self.marker_id == 0:
            rospy.loginfo('y error trash can ')
            rospy.loginfo(y-self.y_distance_trashcan)
            if abs(x) < self.x_margin_trashcan and abs(y-self.y_distance_trashcan) < self.y_error_trashcan:
                return 1
            else:
                return 0


def call_robotarm_service():
    rospy.loginfo("waiting for server")
    rospy.wait_for_service('call_arm')
    
    try:
        call = SetBool()
        call = True
        arm_calling = rospy.ServiceProxy('call_arm', SetBool)
        res = arm_calling(call).success
        rospy.loginfo(res)
        return res
    except rospy.ServiceException as e:
        print("Service call failed : ", e)
    

if __name__ == "__main__":
    rospy.init_node('marker_follower', anonymous=True)
    mf = marker_follower()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    