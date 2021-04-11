#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist

import matplotlib.pyplot as plt
import numpy as np


print("kuboki command")

l=0.120
r=0.025
modelo=np.array([[1,0,-l],[1,0,l],[0,1,0]])
print("modelo matricial: ",modelo)
right_pub = rospy.Publisher('right_wheel_ctrl/command', Float64, queue_size=10)
left_pub = rospy.Publisher('left_wheel_ctrl/command', Float64, queue_size=10)

def callback(data):
    rospy.loginfo('Commando recibido, velocidad en:\n %s', data)
    vel_cart=np.array([data.linear.x,data.linear.y,data.angular.z])
    vel_rad=np.matmul(modelo,vel_cart)
    vels=vel_rad/r
    print("velocidades ruedas: ",vels)
    right_pub.publish(vels[0])
    left_pub.publish(vels[1])
def listener():
    rospy.init_node('kuboki_model', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def main():
  try:
      print("iniciando kuboki control")
      listener()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
