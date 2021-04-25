#!/usr/bin/env python
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Twist, TransformStamped
from std_msgs.msg import String, Header
from math import pi, radians
from gazebo_msgs.msg import ModelState, ModelStates
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster
import rospy

odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
obj="taller1_bot"
odom_msg=Odometry()
h = Header()
tb = TransformBroadcaster()
odom_trans=TransformStamped()
rospy.init_node('robot_odom', anonymous=True)
rate = rospy.Rate(50) # 10hz

def get_odom(modelos):
    index=modelos.name.index(obj)
    pose=modelos.pose[index]
    vel=modelos.twist[index]

    odom_trans.transform.translation.x=pose.position.x;
    odom_trans.transform.translation.y=pose.position.y;
    odom_trans.transform.translation.z=0;
    odom_trans.transform.rotation=pose.orientation;

    odom_msg.pose.pose.position.x=pose.position.x;
    odom_msg.pose.pose.position.y=pose.position.y;
    odom_msg.pose.pose.position.z=0
    odom_msg.pose.pose.orientation=pose.orientation;
    odom_msg.twist.twist.linear.x=vel.linear.x;odom_msg.twist.twist.linear.y=vel.linear.y;
    odom_msg.twist.twist.angular.z=vel.angular.z;

def pub_odom():
    h.frame_id="odom"
    odom_trans.header=h
    odom_trans.child_frame_id="base_footprint"
    odom_msg.header=h
    odom_msg.child_frame_id="base_footprint"
    while not rospy.is_shutdown():
        # print(odom_msg)
        h.stamp = rospy.Time.now()
        tb.sendTransformMessage(odom_trans)
        odom_pub.publish(odom_msg)
        rate.sleep()

def suscriber():
    rospy.Subscriber('/gazebo/model_states',ModelStates,get_odom)
    pub_odom()
    rospy.spin()

if __name__ == '__main__':
    try:
        print("iniciando true odometria")
        suscriber()
    except rospy.ROSInterruptException:
        pass
