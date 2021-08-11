import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

rospy.init_node('image_taker', anonymous=True)
rate = rospy.Rate(1) # 10hz
rate.sleep()
print("iniciado")
bridge = CvBridge()


for i in range(10):
    print('next:')
    _ = input()
    data = rospy.wait_for_message("/camera/image_raw", Image)
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (7,7), None)
    if ret==True:
        print("chess encontrado")
        # cv2.imshow('img', gray)
        # cv2.waitKey(0)
        print('num:')
        i = input()
        cv2.imwrite("cuadros"+i+".png", gray)
    else:
        print("no cuadros")
# Find the chess board corners
