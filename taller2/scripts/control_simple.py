#!/usr/bin/env python
from tf.transformations import quaternion_from_euler, euler_from_quaternion, compose_matrix
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String, Header
from math import pi, radians, degrees, sqrt, atan2, cos, sin
from nav_msgs.msg import Odometry
import numpy as np
import rospy

twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
rospy.init_node('controlador_cartesiano', anonymous=True)
rate = rospy.Rate(10) # 10hz
pose=Pose()
twist=Twist()
def get_odom(odom_msg):
    pose.position.x=odom_msg.pose.pose.position.x
    pose.position.y=odom_msg.pose.pose.position.y
    pose.orientation=odom_msg.pose.pose.orientation

def controller():
    rospy.sleep(1)
    print("en el controler")
    k_v=0.4;k_w=1;
    x_goal=-3;y_goal=2;theta_goal=0
    while not rospy.is_shutdown():
        print("true goal:", x_goal, y_goal)
        (roll_bot,pitch_bot,yaw_bot)=euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
        orient = np.array([roll_bot,pitch_bot,yaw_bot])
        trans = np.array([pose.position.x,pose.position.y,0])
        T_bot = compose_matrix(angles=orient, translate=trans)
        T_bot_inv=np.linalg.inv(T_bot)

        orient = np.array([0,0,0])
        trans = np.array([x_goal,y_goal,0])
        T_punto = compose_matrix(angles=orient, translate=trans)

        T_punto_bot=np.matmul(T_bot_inv,T_punto)
        punto_x=T_punto_bot[0][3];punto_y=T_punto_bot[1][3]
        print("goal from bot:", punto_x, punto_y)

        giro=atan2(punto_y,punto_x)
        print("angulo_a_girar: ", degrees(giro))
        print("\n")

        if ((sqrt((pose.position.x-x_goal)**2+(pose.position.y-y_goal)**2))<0.01):
            twist.linear.x=0
            twist.angular.z=0
        else:
            twist.angular.z=k_w*(giro)
            twist.linear.x=k_v*sqrt((pose.position.x-x_goal)**2+(pose.position.y-y_goal)**2)

        if(twist.linear.x>0.3): #control velocidad
            twist.linear.x=0.3
        if(twist.angular.z>1): #control velocidad angular
            twist.angular.z=1
        if(twist.angular.z<(-1)): #control velocidad angular
            twist.angular.z=1*-1

        twist_pub.publish(twist)
        print(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        print("iniciando controllador cartesiano")
        rospy.Subscriber('/odom',Odometry,get_odom)
        controller()
        rospy.spin()
        # suscriber()
    except rospy.ROSInterruptException:
        pass
