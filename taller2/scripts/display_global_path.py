#!/usr/bin/env python
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Twist, TransformStamped
from std_msgs.msg import String, Header
from math import pi, radians
from gazebo_msgs.msg import ModelState, ModelStates
from nav_msgs.msg import Odometry, Path
from tf import TransformBroadcaster
import rospy

obj="taller1_bot"
h = Header()
odoms=[0,0,0,0,0,0]
def get_odom(odom):
    # odoms[0]=odom.pose.pose.position.x
    # if (abs(odoms[0])-abs(odoms[1])>0.01):
    if (1-0>0.01):
        print(odoms)
        print("si cambio pss")
        odoms[1]=odoms[0]

def suscriber():
    rospy.init_node('path_painter', anonymous=True)
    rospy.Subscriber('/odom',Odometry,get_odom)
    rospy.spin()

if __name__ == '__main__':
    try:
        print("dibujando el path en rviz")
        suscriber()
    except rospy.ROSInterruptException:
        pass
