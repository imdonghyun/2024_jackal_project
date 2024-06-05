#!/usr/bin/env python3
import numpy as np
from math import *
import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool

# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116
ADDR_MOVING_SPEED       = 32
ADDR_TORQUE_LIMIT       = 34
ADDR_PRESENT_POSITION   = 132
ADDR_PRESENT_CURRENT    = 126
ADDR_MOVING             = 122
ADDR_GOAL_CURRENT       = 102

# Data Byte Length
LEN_GOAL_POSITION       = 4
LEN_PRESENT_POSITION    = 4
LEN_PRESENT_SPEED       = 4
LEN_MOVING              = 1

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL11 = 11
DXL12 = 12
DXL13 = 13
DXL14 = 14
DXL15 = 15

BAUDRATE                    = 1000000            # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 40             # Dynamixel moving status threshold

MOTOR_SPEED = 300
CURRENT_LIMIT = 70

PI = np.pi

motors = [DXL11, DXL12, DXL13, DXL14]

camera_offset = np.array([-60, -40, -20])
object_offset = np.array([0, 10, 0])
default_p = np.array([0, 160, 160])

import os
from time import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

position_groupBulkRead = GroupBulkRead(portHandler, packetHandler)

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

#Torque On
def torque_on():
    for id in motors:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    packetHandler.write1ByteTxRx(portHandler, DXL15, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    print('\033[32m' + 'Torque On' + '\033[0m')

#Torque Off
def torque_off():
    for id in motors:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    packetHandler.write1ByteTxRx(portHandler, DXL15, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    print('\033[31m' + 'Torque Off' + '\033[0m')

l1 = 76.5
l2 = sqrt(128**2 + 24**2)
l3 = 124
l4 = 130

alpha = atan(128/24)
beta = PI/2 - alpha

def sat(x, minimum, maximum):
    if x<minimum:
        return minimum
    elif x>maximum:
        return maximum
    else:
        return x

def calc(p, hor):
    px = p[0]
    py = p[1]
    pz = p[2]

    X = sqrt(px**2 + py**2)
    Y = pz + l4 - l1

    if hor:
        X = sqrt(px**2 + py**2) - l4
        Y = pz - l1
        
    L = sqrt(X**2 + Y**2)
    c3 = (L**2 - l2**2 - l3**2)/(2*l2*l3)
    c3 = (1 if c3 > 1 else c3)
    s3 = sqrt(1-c3**2)
    k1 = l2 + l3*c3
    k2 = l2*s3

    theta1 = atan2(-px, py)
    theta2 = sat(atan2(k1*X-k2*Y, k1*Y+k2*X) - beta, -PI/2, PI/2)
    theta3 = sat(acos(c3) - alpha, -PI/2, PI/2)
    theta4 = PI - (theta2+theta3+alpha+beta)
    
    if hor:
        theta4 = theta4 - PI/2

    theta = np.array([theta1, theta2, theta3, theta4])

    return theta

def rad2pos(rad):
    return (int)(2048 + rad*2048/PI)

def syncAdd(ID, pos):
    param = [DXL_LOBYTE(DXL_LOWORD(pos)), 
            DXL_HIBYTE(DXL_LOWORD(pos)), 
            DXL_LOBYTE(DXL_HIWORD(pos)), 
            DXL_HIBYTE(DXL_HIWORD(pos))]
    groupSyncWrite.addParam(ID, param)

def setDefault():
    theta = calc(default_p, 1)
    moveTo(theta)

def grap():
    packetHandler.write4ByteTxRx(portHandler, DXL15, ADDR_GOAL_POSITION, 2600)
    time.sleep(2)
    cur,_,_ = packetHandler.read2ByteTxRx(portHandler, DXL15, ADDR_PRESENT_CURRENT)
    print(cur)
    if cur>CURRENT_LIMIT-10 and cur<1000:
        return 1
    else:
        return 0
    

def get_joint_pos():
    pos = np.zeros(4)
    for idx, i in enumerate(motors):    
        pos[idx],_,_ = packetHandler.read4ByteTxRx(portHandler, i, ADDR_PRESENT_POSITION)    
    return pos

def moveTo(theta):
    step = 20
    step_diff = np.zeros(4)
    ppos = get_joint_pos()

    for i in range(4):
        goal = rad2pos(theta[i])
        step_diff[i] = (goal - ppos[i])/step

    for i in range(step):
        for idx, j in enumerate(motors):
            syncAdd(j, (int)(ppos[idx] + step_diff[idx]*i))
        groupSyncWrite.txPacket()
        groupSyncWrite.clearParam()
        time.sleep(0.05)
    
def carrying(req):
    global grip
    if not grip:
        rospy.loginfo("catching process")
        p = np.array([0, 190, -80]) - camera_offset
        p0 = p + np.array([0, 0, 100])
        theta0 = calc(p0, 0)
        theta = calc(p, 0)
        moveTo(theta0)
        packetHandler.write4ByteTxRx(portHandler, DXL15, ADDR_GOAL_POSITION, 1024)
        time.sleep(0.2)
        moveTo(theta)
        time.sleep(0.2)
        catch = grap()           
        if catch:
            p = np.array([-160, 0, 74.5])
            theta = calc(p, 0)
            moveTo(theta0)
            moveTo(theta)
            grip = True
            print("success")
            return True, ""
        else:
            setDefault()
            packetHandler.write4ByteTxRx(portHandler, DXL15, ADDR_GOAL_POSITION, 1024)
            time.sleep(1)
            print("failed")
            return False, ""
    if grip:
        rospy.loginfo("putting process")
        p = np.array([0, 250, 160]) - camera_offset
        p0 = np.array([-250, 0, 160]) - camera_offset
        theta0 = calc(p0, 1)
        theta = calc(p, 1)
        moveTo(theta0)
        packetHandler.write4ByteTxRx(portHandler, DXL14, ADDR_GOAL_POSITION, 1024)
        theta4 = theta[3]
        theta[3] = -PI/2
        time.sleep(0.5)
        moveTo(theta)
        packetHandler.write4ByteTxRx(portHandler, DXL14, ADDR_GOAL_POSITION, rad2pos(theta4))
        time.sleep(0.5)
        packetHandler.write4ByteTxRx(portHandler, DXL15, ADDR_GOAL_POSITION, 1024)
        grip = False
        print("release")
        time.sleep(0.5)
        setDefault()
        return True, ""
        


def robotarm_server():
    rospy.init_node('robotarm_server')
    s = rospy.Service('call_arm', SetBool, carrying)


    rospy.spin()
        

if __name__ == '__main__':
    global grip
    grip = False
    torque_on()
    setDefault()
    # cur = packetHandler.read2ByteTxRx(portHandler, DXL15, ADDR_PRESENT_CURRENT)
    # print(cur)
    packetHandler.write2ByteTxRx(portHandler, DXL15, ADDR_GOAL_CURRENT, CURRENT_LIMIT)
    packetHandler.write4ByteTxRx(portHandler, DXL15, ADDR_GOAL_POSITION, 1024)
    
    rospy.init_node('robotarm_server')
    s = rospy.Service('call_arm', SetBool, carrying)
    cur_observer = rospy.Publisher('current', Int32, queue_size=1)
    rospy.spin()
    # while not rospy.is_shutdown():
    #     cur = Int32()
    #     cur.data = packetHandler.read2ByteTxRx(portHandler, DXL15, ADDR_PRESENT_CURRENT)
    #     cur_observer.publish(cur)
