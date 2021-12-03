#!/usr/bin/env python

import rospy
import math
import sys
import signal
from time import time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from util import rotateQuaternion, getHeading

XY_TOLERANCE = 0.5
THETA_TOLERANCE = math.pi / 2
MAX_TIME = 60.0 # Seconds

def signal_handler(signal, frame):
    print("\nInterrupted")
    sys.exit(1)

signal.signal(signal.SIGINT, signal_handler)

def pub_goal_pose(x, y, theta):
    rospy.init_node('goal_pos')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)
    checkpoint = PoseStamped()

    checkpoint.header.frame_id = "map"

    checkpoint.pose.position.x = x
    checkpoint.pose.position.y = y
    checkpoint.pose.position.z = 0.0

    checkpoint.pose.orientation = rotateQuaternion(Quaternion(w=1), theta)

    pub.publish(checkpoint)

    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                     _robot_pose_callback, queue_size=10)

    theta = getHeading(checkpoint.pose.orientation)

    global robot_x, robot_y, robot_theta
    robot_x = float('inf')
    robot_y = float('inf')
    robot_theta = float('inf')

    init_time = time()
    time_passed = 0
    rate = rospy.Rate(1)
    while time_passed < MAX_TIME and \
        (abs(robot_x - x) > XY_TOLERANCE or
         abs(robot_y - y) > XY_TOLERANCE or
         abs(robot_theta - theta) > THETA_TOLERANCE):

        # print("x_diff: ", abs(robot_x - x))
        # print("y_diff: ", abs(robot_y - y))
        # print("theta_diff: ", abs(robot_theta - theta))
        # print("robot_theta: ", robot_theta)
        # print("theta: ", theta)
        # print()
        time_passed = time() - init_time
        rate.sleep()

    if time_passed > MAX_TIME:
        print("Time Ran Out")
        return False
    else:
        rospy.sleep(2)
        print("Goal Reached")
        return True


def _robot_pose_callback(pose):
    global robot_x, robot_y, robot_theta
    robot_x = pose.pose.pose.position.x
    robot_y = pose.pose.pose.position.y
    robot_theta = getHeading(pose.pose.pose.orientation)


if __name__ == '__main__':
    try:
        pub_goal_pose(9.0, 9.0, -math.pi /2)
    except rospy.ROSInterruptException:
        pass
