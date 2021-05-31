#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
from kmr_iiwa_commander.msg import PoseRPY, Num, PoseRPYarray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import pi, radians
import copy

import rospy

if __name__ == '__main__':
    pub_pose = rospy.Publisher('robot_commander/cmd_pose', PoseRPY, queue_size=10)
    pub_joints = rospy.Publisher('robot_commander/cmd_joints', Num, queue_size=10)
    pub_path = rospy.Publisher('robot_commander/cmd_path', PoseRPYarray, queue_size=10)
    pub_vacuum = rospy.Publisher('vacuum_commander/cmd', std_msgs.msg.String, queue_size=10)
    rospy.init_node('poseSender', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    rate.sleep()

    # punto=PoseRPY()
    # punto.position.x=0.50;punto.position.y=-0.2;punto.position.z=0.2
    # punto.rpy.roll=0;punto.rpy.pitch=pi/2;punto.rpy.yaw=0
    # pub_pose.publish(punto)
    # msg=rospy.wait_for_message('robot_commander/pose_done',std_msgs.msg.String)

    joints=Num()

    #-----------------------------------------------------------------
    joints.joints=[-1.54,0.98,-0.11,1.22,-2.97,-1,0]
    pub_joints.publish(joints)
    msg=rospy.wait_for_message('robot_commander/joint_done',std_msgs.msg.String)
    #
    # joints.joints=[radians(-47),radians(61),radians(26),radians(114),radians(-32),radians(-71),radians(-15)]
    # pub_joints.publish(joints)
    # msg=rospy.wait_for_message('robot_commander/joint_done',std_msgs.msg.String)
    #
    # lista=PoseRPYarray()
    # lista.eef_step=0.05
    # punto=PoseRPY()
    # punto.position.x=0.55;punto.position.y=0.2;punto.position.z=-0.05
    # punto.rpy.roll=0;punto.rpy.pitch=pi/2;punto.rpy.yaw=0
    # lista.poses.append(copy.deepcopy(punto))
    # punto.position.y=-0.5;
    # lista.poses.append(copy.deepcopy(punto))
    # pub_path.publish(lista)
    # msg=rospy.wait_for_message('robot_commander/path_done',std_msgs.msg.String)
    #
    # lista=PoseRPYarray()
    # lista.eef_step=0.05
    # punto.position.z=0.31;
    # lista.poses.append(copy.deepcopy(punto))
    # punto.position.y=0.2
    # lista.poses.append(copy.deepcopy(punto))
    # punto.position.z=0.66;
    # lista.poses.append(copy.deepcopy(punto))
    # punto.position.y=-0.4
    # lista.poses.append(copy.deepcopy(punto))
    #
    # pub_path.publish(lista)
    # msg=rospy.wait_for_message('robot_commander/path_done',std_msgs.msg.String)
    #
    # msg=rospy.wait_for_message('gazebo/gripper/gripped',std_msgs.msg.String)
    # print("objeto en tcp ", msg.data, " desattaching")
    # pub_vacuum.publish("drop")
    #---------------------------------------------------------
    joints.joints=[radians(-47),radians(61),radians(26),radians(114),radians(-32),radians(-71),radians(-15)]
    pub_joints.publish(joints)
    msg=rospy.wait_for_message('robot_commander/joint_done',std_msgs.msg.String)

    #caja medio
    lista=PoseRPYarray()
    lista.eef_step=0.05
    punto=PoseRPY()
    punto.position.x=0.60;punto.position.y=0.05;punto.position.z=0.3
    punto.rpy.roll=0;punto.rpy.pitch=pi/2;punto.rpy.yaw=0
    lista.poses.append(copy.deepcopy(punto))
    punto.position.x=0.7
    lista.poses.append(copy.deepcopy(punto))
    pub_path.publish(lista)
    msg=rospy.wait_for_message('robot_commander/path_done',std_msgs.msg.String)

    msg=rospy.wait_for_message('gazebo/gripper/gripped',std_msgs.msg.String)
    print("objeto en tcp ", msg.data, " attaching")
    pub_vacuum.publish("suck")

    lista=PoseRPYarray()
    lista.eef_step=0.05
    punto.position.x=0.5;punto.position.z=0.4;
    lista.poses.append(copy.deepcopy(punto))
    pub_path.publish(lista)
    msg=rospy.wait_for_message('robot_commander/path_done',std_msgs.msg.String)

    joints.joints=[-1.54,0.98,-0.11,1.22,-2.97,-1,0]
    pub_joints.publish(joints)
    msg=rospy.wait_for_message('robot_commander/joint_done',std_msgs.msg.String)

    lista=PoseRPYarray()
    lista.eef_step=0.05
    punto=PoseRPY()
    punto.position.x=-0.3;punto.position.y=-0.7;punto.position.z=0.3
    punto.rpy.roll=pi;punto.rpy.pitch=0;punto.rpy.yaw=0
    lista.poses.append(copy.deepcopy(punto))
    pub_path.publish(lista)
    msg=rospy.wait_for_message('robot_commander/path_done',std_msgs.msg.String)

    msg=rospy.wait_for_message('gazebo/gripper/gripped',std_msgs.msg.String)
    print("objeto en tcp ", msg.data, " desattaching")
    pub_vacuum.publish("drop")

    joints.joints=[radians(-47),radians(61),radians(26),radians(114),radians(-32),radians(-71),radians(-15)]
    pub_joints.publish(joints)
    msg=rospy.wait_for_message('robot_commander/joint_done',std_msgs.msg.String)

    #caja abajo
    lista=PoseRPYarray()
    lista.eef_step=0.05
    punto=PoseRPY()
    punto.position.x=0.60;punto.position.y=-0.25;punto.position.z=-0.04
    punto.rpy.roll=0;punto.rpy.pitch=pi/2;punto.rpy.yaw=0
    lista.poses.append(copy.deepcopy(punto))
    punto.position.x=0.7
    lista.poses.append(copy.deepcopy(punto))
    pub_path.publish(lista)
    msg=rospy.wait_for_message('robot_commander/path_done',std_msgs.msg.String)

    msg=rospy.wait_for_message('gazebo/gripper/gripped',std_msgs.msg.String)
    print("objeto en tcp ", msg.data, " attaching")
    pub_vacuum.publish("suck")

    lista=PoseRPYarray()
    lista.eef_step=0.05
    punto.position.x=0.5
    lista.poses.append(copy.deepcopy(punto))
    pub_path.publish(lista)
    msg=rospy.wait_for_message('robot_commander/path_done',std_msgs.msg.String)

    joints.joints=[-1.54,0.98,-0.11,1.22,-2.97,-1,0]
    pub_joints.publish(joints)
    msg=rospy.wait_for_message('robot_commander/joint_done',std_msgs.msg.String)

    lista=PoseRPYarray()
    lista.eef_step=0.05
    punto=PoseRPY()
    punto.position.x=-0.1;punto.position.y=-0.7;punto.position.z=0.3
    punto.rpy.roll=pi;punto.rpy.pitch=0;punto.rpy.yaw=0
    lista.poses.append(copy.deepcopy(punto))
    pub_path.publish(lista)
    msg=rospy.wait_for_message('robot_commander/path_done',std_msgs.msg.String)

    msg=rospy.wait_for_message('gazebo/gripper/gripped',std_msgs.msg.String)
    print("objeto en tcp ", msg.data, " desattaching")
    pub_vacuum.publish("drop")

    joints.joints=[radians(-47),radians(61),radians(26),radians(114),radians(-32),radians(-71),radians(-15)]
    pub_joints.publish(joints)
    msg=rospy.wait_for_message('robot_commander/joint_done',std_msgs.msg.String)

    #caja arriba
    lista=PoseRPYarray()
    lista.eef_step=0.05
    punto=PoseRPY()
    punto.position.x=0.60;punto.position.y=-0.25;punto.position.z=0.64
    punto.rpy.roll=0;punto.rpy.pitch=pi/2;punto.rpy.yaw=0
    lista.poses.append(copy.deepcopy(punto))
    punto.position.x=0.7
    lista.poses.append(copy.deepcopy(punto))
    pub_path.publish(lista)
    msg=rospy.wait_for_message('robot_commander/path_done',std_msgs.msg.String)

    msg=rospy.wait_for_message('gazebo/gripper/gripped',std_msgs.msg.String)
    print("objeto en tcp ", msg.data, " attaching")
    pub_vacuum.publish("suck")

    lista=PoseRPYarray()
    lista.eef_step=0.05
    punto.position.x=0.5
    lista.poses.append(copy.deepcopy(punto))
    pub_path.publish(lista)
    msg=rospy.wait_for_message('robot_commander/path_done',std_msgs.msg.String)

    joints.joints=[-1.54,0.98,-0.11,1.22,-2.97,-1,0]
    pub_joints.publish(joints)
    msg=rospy.wait_for_message('robot_commander/joint_done',std_msgs.msg.String)

    lista=PoseRPYarray()
    lista.eef_step=0.05
    punto=PoseRPY()
    punto.position.x=-0.3;punto.position.y=-0.5;punto.position.z=0.3
    punto.rpy.roll=pi;punto.rpy.pitch=0;punto.rpy.yaw=0
    lista.poses.append(copy.deepcopy(punto))
    pub_path.publish(lista)
    msg=rospy.wait_for_message('robot_commander/path_done',std_msgs.msg.String)

    msg=rospy.wait_for_message('gazebo/gripper/gripped',std_msgs.msg.String)
    print("objeto en tcp ", msg.data, " desattaching")
    pub_vacuum.publish("drop")

    joints.joints=[-1.54,0.98,-0.11,1.22,-2.97,-1,0]
    pub_joints.publish(joints)
    msg=rospy.wait_for_message('robot_commander/joint_done',std_msgs.msg.String)

    print("rutina done")
