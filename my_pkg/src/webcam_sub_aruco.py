#!/usr/bin/env python3

# import rospy
# import cv2

# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import time, math
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from my_pkg.msg import marker


bridge = CvBridge()
# -3.27 -> 28.42 about 30 degree


#Camera Intrinsic Matrix_jackal
mat_Intrin = np.array([[598.260197,   0.     , 310.132131],\
           [0.     , 597.148735, 255.880159] ,\
           [0.     ,   0.     ,   1.     ]],dtype = np.float32)
mat_distortion = np.array([0.108155, -0.225225, 0.000171, -0.001372, 0.000000],dtype = np.float32)
rectification_matrix = np.array([[1., 0., 0.],[0., 1., 0.],[ 0., 0., 1.]])
projection_matrix = np.array([[605.685183,   0.     , 309.348676,   0.     ],
           [0.     , 605.304552, 255.939734,   0.]     ,
           [0.     ,   0.     ,   1.     ,   0.     ]])

blue_BGR = (255,0,0)
red_BGR = (0, 0, 255)
green_BGR = (0, 255, 0)

axis = np.float32([[10,0,0], [0,10,0], [0,0,-10]]).reshape(-1,3)

before_xyz = (0, 0, 0)
real_marker_size = 25

def camaxis_to_worldaxis(pose):
    x,y,z,rx,ry,rz = pose
    x,y,z = x, z, -y
    rx, ry, rz = rx, rz, -ry

    return (x,y,z,rx,ry,rz)

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = "/camera/color/image_raw"
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.marker_vec_pub = rospy.Publisher('marker_vec', marker, queue_size=1)

    def image_callback(self, msg):
        #print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.ret = 1
            print(e)
        else:
            self.marker_data = self.find_aruco_pose(self.cv2_img)
            if self.marker_data != 1:
                ids = self.marker_data[0]
                pose = self.marker_data[1:]

                world_pose = camaxis_to_worldaxis(pose)
                # if pose[0] < 250 and pose[0] > -200:
                # rospy.loginfo("there is a bottle")
                if ids[0,0] <= 1:
                  if world_pose[0] < 250 and world_pose[0] > -200:  # marker publish range checker
                      if world_pose[1] < 1100:
                          self.marker_vector_publisher(world_pose, ids)
  
                  rospy.loginfo(world_pose)
                
            cv2.putText(self.cv2_img, f"Marker size is {real_marker_size}", (30, 30), cv2.FONT_HERSHEY_PLAIN, 1.1, (0, 255, 100), 1)
            cv2.imshow('frame',self.cv2_img)
            cv2.waitKey(3)
    
    def find_aruco_pose(self, img, marker_type=4, total_markers=50, draw=True):
        global before_xyz
        global real_marker_size
        marker_3d_edges = np.array([    [0.,0.,0.],
                                        [0.,real_marker_size,0.],
                                        [real_marker_size,real_marker_size,0.],
                                        [real_marker_size,0.,0.]], dtype = np.float32).reshape((4,1,3))
    ######    
        #gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        key = getattr(aruco, f'DICT_{marker_type}X{marker_type}_{total_markers}')
        arucoDict = aruco.getPredefinedDictionary(key)
        arucoParam = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(arucoDict, arucoParam)
        corners, ids, _ = detector.detectMarkers(img)
        #corners, ids, _ =bridge.cv_to_imgmsg(cv_image, "bgr8")

        for corner in corners:
            corner = np.array(corner).reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corner
            # print(corner)
            topRightPoint    = (int(topRight[0]),      int(topRight[1]))
            topLeftPoint     = (int(topLeft[0]),       int(topLeft[1]))
            bottomRightPoint = (int(bottomRight[0]),   int(bottomRight[1]))
            bottomLeftPoint  = (int(bottomLeft[0]),    int(bottomLeft[1]))
            centerofmarker = (int((topLeft[0] + topRight[0]+bottomRight[0]+bottomLeft[0])/4),
                            int((topLeft[1] + topRight[1]+bottomRight[1]+bottomLeft[1])/4) )
            

            cv2.circle(img, topLeftPoint, 4, blue_BGR, -1)
            cv2.circle(img, topRightPoint, 4, blue_BGR, -1)
            cv2.circle(img, bottomRightPoint, 4, blue_BGR, -1)
            cv2.circle(img, bottomLeftPoint, 4, blue_BGR, -1)
            cv2.circle(img, centerofmarker, 4, red_BGR, -1)
            
            ret, rvec, tvec = cv2.solvePnP(marker_3d_edges, corner, mat_Intrin, mat_distortion)
            if ret:
                corner = np.array(corner).reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corner
                # print('tvec = ', tvec)
                # print('rvec = ', rvec)

                #origin
                x1 = tvec[0][0]+real_marker_size/2
                y1 = tvec[1][0]+real_marker_size/2
                z1 = tvec[2][0]
                
                
                #rotated, # Camera is truned about 30 degree downward
                x=round(x1,2)
                y=round(y1 * np.cos(math.pi/6) + z1 * np.sin(math.pi/6),2)
                z=round(z1 * np.cos(math.pi/6) - y1*np.sin(math.pi/6),2)
                
                alpha = 0.95
                x,y,z = round(before_xyz[0]*alpha + x*(1-alpha),2),round(before_xyz[1]*alpha + y*(1-alpha),2), round(before_xyz[2]*alpha + z*(1-alpha),2)  
                before_xyz = (x,y,z)
                
                # rospy.loginfo((x1, y1, z1))
                # rospy.loginfo((x, y, z))
                # rospy.loginfo("--")
                rmat = cv2.Rodrigues(rvec)[0]
                
                #version2 matrix transpose
                rx = math.atan2(rmat[1][2], rmat[2][2])
                ry = math.atan2(-rmat[0][2], math.sqrt(rmat[1][2]**2+rmat[2][2]**2))
                rz = math.atan2(rmat[0][1], rmat[0][0]) - math.pi/2

            
                rx=round(-(((np.rad2deg(rx)+360)%360)-180),2)
                ry=round(np.rad2deg(ry),2)
                rz=round(np.rad2deg(rz),2)
                # print(np.rad2deg(rvec[0][0]))
                
                # PnP 결과를 이미지에 그려 확
                text1 = f"{x},{y},{z}" 
                text2 = f"{rx},{ry},{rz}"

                cv2.putText(img, text1, (centerofmarker[0]-40,   centerofmarker[1]+20), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255))
                cv2.putText(img, text2, (centerofmarker[0]-40,   centerofmarker[1]+50), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 1)
                #babo 
                img_shape = img.shape
                centerofImg = (int(img_shape[1]/2), int(img_shape[0]/2))
                end_point = (int(centerofmarker[0]), int(centerofmarker[1]))
                if abs(x) > 5 or abs(y) > 5:
                    cv2.arrowedLine(img, centerofImg, end_point, green_BGR, 5)
                
                imgpts, jac = cv2.projectPoints(axis, rvec, tvec, mat_Intrin, mat_distortion)
            
                a1 = int(tuple(imgpts[0].ravel())[0])
                a2 = int(tuple(imgpts[0].ravel())[1])
                b1 = int(tuple(imgpts[1].ravel())[0])
                b2 = int(tuple(imgpts[1].ravel())[1])
                c1 = int(tuple(imgpts[2].ravel())[0])
                c2 = int(tuple(imgpts[2].ravel())[1])

                cv2.line(img, topLeftPoint, (c1,c2), red_BGR, 2)
                cv2.line(img, topLeftPoint, (a1,a2), blue_BGR, 2)
                cv2.line(img, topLeftPoint, (b1,b2), green_BGR, 2)
                    

                return ids, x,y,z,rx,ry,rz
        return 1
    def marker_vector_publisher(self, pose, ids):
        msg = marker()
        msg.id = ids[0,0]
        msg.x = pose[0]
        msg.y = pose[1]
        msg.z = pose[2] 
        # if pose[0] < 250 and pose[0] > -200:
        #     rospy.loginfo("there is a bottle")
        #TODO Angular data
        # msg.angular.x = pose[4]
        # msg.angular.y = pose[5]
        # msg.angular.z = pose[6]
        self.marker_vec_pub.publish(msg)


def main():
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)

    # Define your image topic
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
    # Set up your subscriber and define its callback
    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    main()
