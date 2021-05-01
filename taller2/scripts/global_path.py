#!/usr/bin/env python
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Twist, TransformStamped, PoseStamped
from std_msgs.msg import String, Header
from math import pi, radians
from gazebo_msgs.msg import ModelState, ModelStates
from nav_msgs.msg import Odometry, Path
from tf import TransformBroadcaster
import rospy
import copy

path_pub = rospy.Publisher('/global_path', Path, latch=True, queue_size=50)
puntos=[[-0.6,0],[-3.5,0],[-3.5,3.5],[1.5,3.5],[1.5,-1.5],[3.5,-1.5],[3.5,-8.0],[-2.5,-8.0],[-2.5,-5.5],[1.5,-5.5],[1.5,-3.5],[-1,-3.5]]
# puntos=[[-3.5,0],[-3.5,3.5]]
rospy.init_node('robot_path', anonymous=True)
rate = rospy.Rate(1) # 10hz
h = Header()
path=Path()
pose_t=PoseStamped()
if __name__ == '__main__':
    try:
        print("dibujando el path en rviz")
        h.frame_id="odom"
        path.header=h
        pose_t.header=h
        while not rospy.is_shutdown():
            # h = Header()
            # path=Path()
            # pose_t=PoseStamped()
            # h.frame_id="odom"
            # path.header=h
            # pose_t.header=h
            for i in range(len(puntos)):
                pose_t.header.stamp = rospy.Time.now()
                pose_t.pose.position.x=puntos[i][0]
                pose_t.pose.position.y=puntos[i][1]
                pose_t.pose.orientation.w=1
                path.poses.append(copy.deepcopy(pose_t))

            print(path)
            path_pub.publish(path)
            rate.sleep()
            break

    except rospy.ROSInterruptException:
        pass
