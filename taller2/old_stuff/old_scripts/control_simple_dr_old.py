#!/usr/bin/env python
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, Twist, TransformStamped
from std_msgs.msg import String, Header
from math import pi, radians, degrees, sqrt, atan2, cos, sin
from gazebo_msgs.msg import ModelState, ModelStates
from nav_msgs.msg import Odometry

import rospy
from dynamic_reconfigure.server import Server
from taller2.cfg import dynamic_paramConfig

twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
rospy.init_node('controlador_cartesiano', anonymous=True)
rate = rospy.Rate(10) # 10hz
pose=Pose()
twist=Twist()

class Controller_dyn():

    def get_odom(self,odom_msg):
        pose.position.x=odom_msg.pose.pose.position.x
        pose.position.y=odom_msg.pose.pose.position.y
        pose.orientation=odom_msg.pose.pose.orientation

    def controller(self):
        rospy.sleep(1)
        print("en el controler")
        k_v=0.4;k_w=1;
        max_v=0.3;max_w=1
        x_goal=-3;y_goal=2;
        while not rospy.is_shutdown():
            x_goal=self.datos["x_goal"];y_goal=self.datos["y_goal"]
            max_v=self.datos["max_v"];max_w=self.datos["max_w"]
            k_v=self.datos["k_v"];k_w=self.datos["k_w"];
            print("x_goal:",self.datos["x_goal"],", y_goal:",self.datos["y_goal"],
                ", max_v:",self.datos["max_v"],", max_w:",self.datos["max_w"],", k_v:",self.datos["k_v"],", k_w:",self.datos["k_w"])
            goal_from_bot=[x_goal-pose.position.x,y_goal-pose.position.y]
            print("goal_from_bot :", goal_from_bot)
            (roll_bot,pitch_bot,yaw_bot)=euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
            print("orientacion_bot :", degrees(yaw_bot))
            theta=atan2(goal_from_bot[1],goal_from_bot[0])
            print("angulo_punto_al_bot: ", degrees(theta))
            if (theta>0 and yaw_bot<0 and x_goal<0):
                yaw_bot=yaw_bot+2*pi
            if (theta<0 and yaw_bot>0 and x_goal<0):
                yaw_bot=(2*pi-yaw_bot)*-1
            giro=theta-yaw_bot
            print("angulo_a_girar: ", degrees(giro))
            # print("\n")
            if(giro>radians(180)):
                giro=(2*pi-giro)*-1
            elif(giro<radians(-180)):
                giro=(-2*pi-giro)*-1
            print("angulo_a_girar2: ", degrees(giro))

            if ((sqrt((pose.position.x-x_goal)**2+(pose.position.y-y_goal)**2))<0.01):
                twist.linear.x=0
                twist.angular.z=0
            else:
                twist.angular.z=k_w*(giro)
                twist.linear.x=k_v*sqrt((pose.position.x-x_goal)**2+(pose.position.y-y_goal)**2)

            if(twist.linear.x>max_v): #control velocidad
                twist.linear.x=max_v
            if(twist.angular.z>max_w): #control velocidad angular
                twist.angular.z=max_w
            if(twist.angular.z<(-max_w)): #control velocidad angular
                twist.angular.z=max_w*-1

            twist_pub.publish(twist)
            print(twist)
            rate.sleep()

    def callback(self, config, level):
        # rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\
        #       {str_param}, {bool_param}""".format(**config))
        # rospy.loginfo("""Reconfigure Request: {x_goal},{y_goal}""".format(**config))
        # print(config)
        self.datos=config
        return config

    def init(self):
        print("iniciando controllador cartesiano")
        rospy.Subscriber('/odom',Odometry,self.get_odom)
        srv = Server(dynamic_paramConfig, self.callback)
        self.controller()
        rospy.spin()


if __name__ == '__main__':
    try:
        contro=Controller_dyn()
        contro.init()

    except rospy.ROSInterruptException:
        pass
