#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from util import rotateQuaternion

def pub_goal_pose(x, y, theta):
    rospy.init_node('goal_pos')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 10)
    rospy.sleep(1)
    checkpoint = PoseStamped()

    checkpoint.header.frame_id = "map"

    checkpoint.pose.position.x = x
    checkpoint.pose.position.y = y
    checkpoint.pose.position.z = 0.0

    checkpoint.pose.orientation = rotateQuaternion(Quaternion(w=1), theta)

    pub.publish(checkpoint)

if __name__ == '__main__':
    try:
        pub_goal_pose(0.0, 0.0, 0.0)
    except rospy.ROSInterruptException:
        pass