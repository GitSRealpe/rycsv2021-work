#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
from kmr_iiwa_commander.msg import PoseRPY, Num
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import pi

import rospy

if __name__ == '__main__':
    pub_pose = rospy.Publisher('robot_commander/cmd_pose', PoseRPY, queue_size=10)
    pub_joints = rospy.Publisher('robot_commander/cmd_joints', Num, queue_size=10)
    pub_vacuum = rospy.Publisher('vacuum_commander/cmd', std_msgs.msg.String, queue_size=10)
    rospy.init_node('poseSender', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    rate.sleep()

    joints=Num()
    # joints.joints=[-0.87,0.81,2.69,-2.03,0.40,1.13]
    # pub_joints.publish(joints)
    # msg=rospy.wait_for_message('robot_commander/joint_done',std_msgs.msg.String)

    joints.joints=[0,0,0,0,0,0,0]
    pub_joints.publish(joints)
    msg=rospy.wait_for_message('robot_commander/joint_done',std_msgs.msg.String)

    joints.joints=[0,-0.71,2.97,1.72,-0.12,0.75,0]
    pub_joints.publish(joints)
    msg=rospy.wait_for_message('robot_commander/joint_done',std_msgs.msg.String)

    joints.joints=[1.72,-0.59,2.79,2.09,-0.12,-1,0]
    pub_joints.publish(joints)
    msg=rospy.wait_for_message('robot_commander/joint_done',std_msgs.msg.String)

    punto=PoseRPY()
    punto.position.x=0.25;punto.position.y=-0.5;punto.position.z=0.3
    punto.rpy.roll=pi/2;punto.rpy.pitch=0;punto.rpy.yaw=0
    pub_pose.publish(punto)
    msg=rospy.wait_for_message('robot_commander/pose_done',std_msgs.msg.String)

    punto=PoseRPY()
    punto.position.x=0.25;punto.position.y=-0.82;punto.position.z=0.3
    punto.rpy.roll=pi/2;punto.rpy.pitch=0;punto.rpy.yaw=0
    pub_pose.publish(punto)
    msg=rospy.wait_for_message('robot_commander/pose_done',std_msgs.msg.String)

    # msg=rospy.wait_for_message('gazebo/gripper/gripped',std_msgs.msg.String)
    # print("objeto en tcp ", msg.data, " attaching")
    # pub_vacuum.publish("suck")

    punto.position.x=0.2;punto.position.y=-0.45;punto.position.z=0.4
    punto.rpy.roll=pi/2;punto.rpy.pitch=0;punto.rpy.yaw=-pi/2
    pub_pose.publish(punto)
    msg=rospy.wait_for_message('robot_commander/pose_done',std_msgs.msg.String)

    punto.position.x=-0.58;punto.position.y=-0.34;punto.position.z=0.2
    punto.rpy.roll=pi/2;punto.rpy.pitch=0;punto.rpy.yaw=-pi/2
    pub_pose.publish(punto)
    msg=rospy.wait_for_message('robot_commander/pose_done',std_msgs.msg.String)

    # punto.position.x=-0.7;punto.position.y=0.35;punto.position.z=0.15
    # punto.rpy.roll=pi/2;punto.rpy.pitch=0;punto.rpy.yaw=-pi
    # pub_pose.publish(punto)
    # msg=rospy.wait_for_message('robot_commander/pose_done',std_msgs.msg.String)
    #
    # msg=rospy.wait_for_message('gazebo/gripper/gripped',std_msgs.msg.String)
    # print("objeto en tcp ", msg.data, " attaching")
    # pub_vacuum.publish("drop")

    print("rutina done")
