#!/usr/bin/env python
from tf.transformations import quaternion_from_euler, quaternion_about_axis
from geometry_msgs.msg import Pose, Twist, TransformStamped, PoseStamped
from std_msgs.msg import String, Header
from math import pi, radians
from gazebo_msgs.msg import ModelState, ModelStates
from nav_msgs.msg import Odometry, Path
from tf import TransformBroadcaster
import rospy
import copy

path_pub = rospy.Publisher('/global_path', Path, latch=True, queue_size=50)
# puntos=[[1,0,90],[2,-3,45],[5.5,4.5,0],[-3,3,0],[-4,-2,0]]
puntos=[[5.8,-4.1,180],[4,-4.5,180],[4,-2,90],[4,-2,0],[6,-2,0],[6,2,90],[4.5,2,90],[4,1,0],[3.5,1,0]]
# puntos.reverse()

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
            for i in range(len(puntos)):
                pose_t.header.stamp = rospy.Time.now()
                pose_t.pose.position.x=puntos[i][0]
                pose_t.pose.position.y=puntos[i][1]
                (x,y,z,w)=quaternion_about_axis(radians(puntos[i][2]),(0,0,1))
                pose_t.pose.orientation.x=x
                pose_t.pose.orientation.y=y
                pose_t.pose.orientation.z=z
                pose_t.pose.orientation.w=w
                path.poses.append(copy.deepcopy(pose_t))

            print(path)
            path_pub.publish(path)
            rate.sleep()
            break

    except rospy.ROSInterruptException:
        pass
