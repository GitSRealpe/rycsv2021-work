#!/usr/bin/env python
from tf.transformations import quaternion_from_euler, euler_from_quaternion, compose_matrix
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String, Header
from math import pi, radians, degrees, sqrt, atan2, cos, sin
from nav_msgs.msg import Odometry, Path
import numpy as np
import rospy

twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
rospy.init_node('controlador_cartesiano', anonymous=True)
rate = rospy.Rate(10) # 10hz

pose=Pose()
twist=Twist()

class path_Controller():

    def get_odom(self, odom_msg):
        pose.position.x=odom_msg.pose.pose.position.x
        pose.position.y=odom_msg.pose.pose.position.y
        pose.orientation=odom_msg.pose.pose.orientation

    def get_path(self, path_msg):
        self.traj=path_msg
        print("llego el path")
        self.new=True
        self.controller()

    def controller(self):
        rospy.sleep(1)
        print("en el controler")
        k_v=1;k_w=5;a=1;k_y=1;k_x=1;
        punto=self.traj.poses.pop(0).pose;
        x_goal=punto.position.x
        y_goal=punto.position.y
        (roll_goal,pitch_goal,yaw_goal)=euler_from_quaternion([punto.orientation.x,punto.orientation.y,punto.orientation.z,punto.orientation.w])
        self.new=False
        max_v=0.5
        while not rospy.is_shutdown() and not self.new:
            print("true goal:", x_goal, y_goal)
            (roll_bot,pitch_bot,yaw_bot)=euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])

            trans = np.array([pose.position.x,pose.position.y,0])
            orient = np.array([roll_bot,pitch_bot,yaw_bot])
            T_bot = compose_matrix(angles=orient, translate=trans)
            T_bot_inv=np.linalg.inv(T_bot)
            trans = np.array([x_goal,y_goal,0])
            orient = np.array([roll_goal,pitch_goal,yaw_goal])
            T_punto = compose_matrix(angles=orient, translate=trans)
            T_punto_bot=np.matmul(T_bot_inv,T_punto)
            punto_x=T_punto_bot[0][3];punto_y=T_punto_bot[1][3]
            print("goal from bot:", punto_x, punto_y)

            # giro=atan2(punto_y,punto_x)
            giro=yaw_goal-yaw_bot;
            print("griro bastard: ", degrees(giro))
            if (giro>radians(180)):
                giro=giro-2*pi
            if (giro<radians(-180)):
                giro=(2*pi+giro)
            print("angulo_a_girar: ", degrees(giro))
            # print("\n")

            if ((sqrt((pose.position.x-x_goal)**2+(pose.position.y-y_goal)**2))<0.2 and abs(giro)<radians(10)):
                if (len(self.traj.poses)>0):
                    punto=self.traj.poses.pop(0).pose;
                    x_goal=punto.position.x
                    y_goal=punto.position.y
                    (roll_goal,pitch_goal,yaw_goal)=euler_from_quaternion([punto.orientation.x,punto.orientation.y,punto.orientation.z,punto.orientation.w])
                else:
                    if ((sqrt((pose.position.x-x_goal)**2+(pose.position.y-y_goal)**2))<0.01):
                        giro=0;pose.position.x=x_goal;pose.position.y=y_goal;

            twist.angular.z=k_w*(giro)
            twist.linear.x=k_v*(punto_x)
            twist.linear.y=k_v*(punto_y)

            if(twist.linear.x>max_v): #control velocidad lineal
                k_x=max_v/twist.linear.x
            if(twist.linear.y>max_v): #control velocidad lineal
                k_y=max_v/twist.linear.y
            if(twist.linear.x<-max_v): #control velocidad lineal
                k_x=-max_v/twist.linear.x
            if(twist.linear.y<-max_v): #control velocidad lineal
                k_y=-max_v/twist.linear.y

            a=min(k_x,k_y)
            twist.linear.x=a*twist.linear.x
            twist.linear.y=a*twist.linear.y

            if(twist.angular.z>radians(60)): #control velocidad angular
                twist.angular.z=radians(60)
            if(twist.angular.z<(radians(-60))): #control velocidad angular
                twist.angular.z=radians(-60)
            twist_pub.publish(twist)
            print(twist)
            rate.sleep()


    def init(self):
        print("iniciando controllador cartesiano")
        rospy.Subscriber('/odom', Odometry, self.get_odom)
        rospy.Subscriber('/global_path', Path, self.get_path)
        # controller()
        rospy.spin()
if __name__ == '__main__':
    try:
        ctrl_path=path_Controller()
        ctrl_path.init()
    except rospy.ROSInterruptException:
        pass
