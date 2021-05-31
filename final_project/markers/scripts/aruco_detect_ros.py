#!/usr/bin/env python3
#include <opencv2/aruco.hpp>
from __future__ import print_function
import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage


arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)#selection dictionary
arucoParams = cv2.aruco.DetectorParameters_create()

class Camera_ARU:  

	def __init__(self):
		
		self.cameraTopic = None
		self.encoding = None

        # Load parameters from parameter server
		
		self.getParameters()

        # Check that the parameters where correctly loaded
		if(self.cameraTopic is None or self.encoding is None):
			rospy.signal_shutdown("Parameters not read")
		
		else:
			rospy.loginfo("Parameters found")

        # Create CV bridge
		self.bridge = CvBridge()
		# Create image subscriber
		self.image_sub = rospy.Subscriber(self.cameraTopic, CompressedImage, self.cam_image_cb)

        # Create image publisher
		self.image_pub = rospy.Publisher("image_orb_features", Image, queue_size=10)
        

    #------------------------------------------------------#
    # Function to get parameters
	
	def getParameters(self):
		
		if rospy.has_param('~cam_topic'):   self.cameraTopic = rospy.get_param('~cam_topic')
		if rospy.has_param("~encoding"):    self.encoding = rospy.get_param("~encoding")

    #------------------------------------------------------#
    # Callback function for image
    
	def cam_image_cb(self,data):

		try:
			cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		(corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image,arucoDict, parameters=arucoParams)
		if len(corners)>0:
			ids =ids.flatten()

			for (markerCorner, markerID) in zip(corners, ids):
				corners = markerCorner.reshape((4, 2))
				(topLeft, topRight, bottomRight, bottomLeft) = corners
			# convert each of the (x, y)-coordinate pairs to integers
				topRight = (int(topRight[0]), int(topRight[1]))
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))
			# draw the bounding box of the ArUCo detection
				cv2.line(cv_image, topLeft, topRight, (0, 255, 0), 2)
				cv2.line(cv_image, topRight, bottomRight, (0, 255, 0), 2)
				cv2.line(cv_image, bottomRight, bottomLeft, (0, 255, 0), 2)
				cv2.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 2)
			# compute aids = ids.flatten()nd draw the center (x, y)-coordinates of the
			# ArUco marker
				cX = int((topLeft[0] + bottomRight[0]) / 2.0)
				cY = int((topLeft[1] + bottomRight[1]) / 2.0)
				cv2.circle(cv_image, (cX, cY), 4, (0, 0, 255), -1)
				# draw the ArUco marker ID on the cv_image
				cv2.putText(cv_image, str(markerID),(topLeft[0], topLeft[1] - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
			# show the output cv_ima
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            #self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(cv_image2))
		except CvBridgeError as e:
			print(e)

	#------------------------------------------------------#
    #------------------------------------------------------#
    #------------------------------------------------------#
    

#-----------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------#
if __name__ == '__main__':

    # Firt init the node and then the object to correctly find the parameters
    rospy.init_node('image_features', anonymous=True)
    Camera_ARU()
    
        
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
