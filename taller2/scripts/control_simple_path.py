#!/usr/bin/env python
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, Twist, TransformStamped
from std_msgs.msg import String, Header
from math import pi, radians, degrees, sqrt, atan2, cos, sin
from gazebo_msgs.msg import ModelState, ModelStates
from nav_msgs.msg import Odometry

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
    # print(pose)

def controller():
    # twist_pub.publish(odom_msg)
    rospy.sleep(1)
    print("en el controler")
    k_v=1;k_w=1;
    path=[[-3.5,0],[-3.5,3.5],[1.5,3.5],[1.5,-1.5],[3.5,-1.5],[3.5,-8.0],[-2.5,-8.0],[-2.5,-5.5],[1.5,-5.5],[1.5,-3.5],[-1,3.5]]
    x_goal=path[0][0];y_goal=path[0][1];
    next=0
    while not rospy.is_shutdown():
        goal_from_bot=[x_goal-pose.position.x,y_goal-pose.position.y]
        print("goal_from_bot :", goal_from_bot)
        theta_robot=euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
        print("orientacion_bot :", degrees(theta_robot[2]))
        theta=atan2(goal_from_bot[1],goal_from_bot[0])
        print("angulo_punto_al_bot: ", degrees(theta))
        # if (theta<0):
        #     print("cerca a +-180")
        #     theta=theta+2*pi
        theta=theta-theta_robot[2]
        print("angulo_bot_alpunto: ", degrees(theta))
        print("\n")

        if ((sqrt((pose.position.x-x_goal)**2+(pose.position.y-y_goal)**2))<0.1):
            # twist.linear.x=0
            # twist.angular.z=0
            if (not next>=len(path)):
                x_goal=path[next][0];y_goal=path[next][1];
                next=next+1
            else:
                twist.linear.x=0
                twist.angular.z=0
        else:

            # if (theta>radians(175) or theta<radians(-175)):
            #     theta=2
            twist.angular.z=k_w*(theta)
            twist.linear.x=k_v*sqrt((pose.position.x-x_goal)**2+(pose.position.y-y_goal)**2)

        if(twist.linear.x>0.3): #control velocidad
            twist.linear.x=0.3
        twist_pub.publish(twist)
        print(twist)
        rate.sleep()

def suscriber():
    rospy.Subscriber('/odom',Odometry,get_odom)
    controller()
    rospy.spin()

if __name__ == '__main__':
    try:
        print("iniciando controllador cartesiano")
        suscriber()
    except rospy.ROSInterruptException:
        pass
