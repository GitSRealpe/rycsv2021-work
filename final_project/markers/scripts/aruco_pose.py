import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import tf.transformations as tf
from geometry_msgs.msg import Pose, Twist, TransformStamped
from std_msgs.msg import String, Header
from math import pi, radians
from tf import TransformBroadcaster

#coeficiente de distorcion
# dist=np.array(([[-0.58650416 , 0.59103816, -0.00443272 , 0.00357844 ,-0.27203275]]))
# #matrix
# mtx=np.array([[398.12724231  , 0.      ,   304.35638757],
#  [  0.       ,  345.38259888, 282.49861858],
#  [  0.,           0.,           1.        ]])

 #coeficiente de distorcion
dist=np.array(([[-1.27796217e-03, 9.43000230e-03, -1.21250279e-04,  8.80471279e-05,
  -1.13316181e-02]]))
 #matrix
mtx=np.array([[554.50039606,   0.,         319.51374943],
 [  0.,         554.53463923, 239.40679061],
 [  0.,           0.,           1.        ]])

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)#selection dictionary
arucoParams = cv2.aruco.DetectorParameters_create()
marker_size=0.05

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/postImage",Image,queue_size=50)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.callback)

    self.h = Header()
    self.tb = TransformBroadcaster()
    self.img_tf=TransformStamped()


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape

    cv2.circle(cv_image, (320,240), 10, (0,255,255),-1)

    (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image,arucoDict, parameters=arucoParams)
    frame_markers = cv2.aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)

    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, mtx, dist)
    # print("trans",tvec)
    # print("rot",rvec)

    # cv2.aruco.drawAxis(cv_image,mtx,dist,rvec,tvec,0.1)
    cv2.aruco.drawAxis(frame_markers,mtx,dist,rvec,tvec,0.05)
    cv2.putText(frame_markers,np.array_str(tvec),(0,50),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)

    # cv2.imshow("Image window", frame_markers)
    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(3)

    self.tf_pub(tvec,rvec)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame_markers, "bgr8"))
    except CvBridgeError as e:
      print(e)


  def tf_pub(self,tvec,rvec):
    # q=tf.quaternion_from_euler(0,-pi/2,-pi/2,axes="rxyz")
    # q=tf.quaternion_from_euler(pi/2,0,-pi/2,axes="sxyz")
    q=tf.quaternion_from_euler(rvec[0][0][0],rvec[0][0][1],rvec[0][0][2],axes="sxyz")
    print(rvec)
    self.img_tf.transform.translation.y=-tvec[0][0][0];
    self.img_tf.transform.translation.z=-tvec[0][0][1];
    self.img_tf.transform.translation.x=tvec[0][0][2];
    # self.img_tf.transform.rotation.w=1;
    self.img_tf.transform.rotation.x=q[0];self.img_tf.transform.rotation.y=q[1];
    self.img_tf.transform.rotation.z=q[2];self.img_tf.transform.rotation.w=q[3];


    self.h.frame_id="cam_link"
    self.img_tf.header=self.h
    self.img_tf.child_frame_id="img_tf"
    self.h.stamp = rospy.Time.now()
    self.tb.sendTransformMessage(self.img_tf)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
