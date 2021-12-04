#!/usr/bin/env python

import rospy
import math
import sys
import signal
from time import time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction
from actionlib import SimpleActionClient
from util import rotateQuaternion, getHeading

XY_TOLERANCE = 0.5
ORIENTATION_TOLERANCE = 0.5
MAX_TIME = 60.0  # Seconds

LINEAR_VEL = 0.5
ANGULAR_VEL = 2.0
VEL_PUB_DURATION_LINEAR = 0.5  # Seconds
VEL_PUB_DURATION_ANGULAR = 2.0  # Seconds

global recovery_counter
recovery_counter = 0

def signal_handler(signal, frame):
    """ Helps to interrupt pub_goal_pose using CTRL+C

    Args:
        num_poses (int): the amount of poses to generate. If not
            provided, `self.NUMBER_PREDICTED_READINGS` is used.
    Return:
        (geometry_msgs.msg.PoseArray): random particle poses
    """
    print("\nInterrupted")
    sys.exit(1)


signal.signal(signal.SIGINT, signal_handler)


def cancel_path():
    """ Cancels the current path
    """
    client = SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    client.cancel_all_goals()
    rospy.loginfo("Cancelled goal_pose")


def _move_lin(lin):
    """ Moves linearly
    Args:
        lin (float): the linear velocity
    """
    pub_velocity = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()

    twist.linear.x = lin

    # Connect with the publisher
    while pub_velocity.get_num_connections() < 1:
        pass

    init_time = time()
    duration = 0
    # Publish to the /cmd_vel topic
    while duration < VEL_PUB_DURATION_LINEAR:
        pub_velocity.publish(twist)
        duration = time() - init_time
    rospy.loginfo("Moved Linearly")


def _move_ang(ang):
    """ Moves angularly
    Args:
        ang (float): the angular velocity
    """
    pub_velocity = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()

    twist.angular.z = ang

    # Connect with the publisher
    while pub_velocity.get_num_connections() < 1:
        pass

    init_time = time()
    duration = 0
    # Publish to the /cmd_vel topic
    while duration < VEL_PUB_DURATION_ANGULAR:
        pub_velocity.publish(twist)
        duration = time() - init_time
    rospy.loginfo("Turned")


def move_forward():
    _move_lin(LINEAR_VEL)


def move_backward():
    _move_lin(-LINEAR_VEL)


def turn_left():
    _move_ang(ANGULAR_VEL)


def turn_right():
    _move_ang(-ANGULAR_VEL)


def recovery(x, y, theta):
    """ Cancels the current path, then moves around to get unstuck
    and then calls pub_goal_pose again to the same path
    Args:
        x (float): goal pose x coordinate
        y (float): goal pose y coordinate
        theta (float): goal pose orientation in radians
    """
    rospy.loginfo("Recovery Started")
    cancel_path()

    move_forward()
    turn_right()
    move_forward()
    turn_left()

    rospy.loginfo("Recovery Ended")
    rospy.loginfo("Trying again")
    pub_goal_pose(x, y, theta)


def pub_goal_pose(x, y, theta):
    """ Publishes to the /move_base_simple/goal topic the coordinates of
        a new goal pose
    Args:
        x (float): goal pose x coordinate
        y (float): goal pose y coordinate
        theta (float): goal pose orientation in radians
    Return:
        (bool): True if goal is reached, False if MAX_TIME seconds have passed and 
            the robot did not get to the goal pose
    """
    rospy.init_node('goal_pos')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)
    checkpoint = PoseStamped()

    checkpoint.header.frame_id = "map"

    checkpoint.pose.position.x = x
    checkpoint.pose.position.y = y
    checkpoint.pose.position.z = 0.0

    checkpoint.pose.orientation.x = 0.0
    checkpoint.pose.orientation.y = 0.0
    checkpoint.pose.orientation.w = 1.0
    checkpoint.pose.orientation.z = 0.0

    checkpoint.pose.orientation = rotateQuaternion(
        checkpoint.pose.orientation, theta)

    pub.publish(checkpoint)

    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                     _robot_pose_callback, queue_size=10)

    global robot_x, robot_y, robot_orientation
    robot_x = float('inf')
    robot_y = float('inf')
    robot_orientation = Quaternion()
    robot_orientation.x = float('inf')
    robot_orientation.y = float('inf')
    robot_orientation.w = float('inf')
    robot_orientation.z = float('inf')

    init_time = time()
    time_passed = 0
    rate = rospy.Rate(1)
    while time_passed < MAX_TIME and \
        (abs(robot_x - x) > XY_TOLERANCE or
         abs(robot_y - y) > XY_TOLERANCE or
         abs(robot_orientation.x - checkpoint.pose.orientation.x) > ORIENTATION_TOLERANCE or
         abs(robot_orientation.y - checkpoint.pose.orientation.y) > ORIENTATION_TOLERANCE or
         abs(robot_orientation.w - checkpoint.pose.orientation.w) > ORIENTATION_TOLERANCE or
         abs(robot_orientation.z - checkpoint.pose.orientation.z) > ORIENTATION_TOLERANCE):

        time_passed = time() - init_time
        rate.sleep()

    if time_passed > MAX_TIME:
        rospy.logwarn("Time Ran Out! Cancelling Path")
        global recovery_counter
        recovery_counter += 1
        if recovery_counter < 3:
            recovery(x, y, theta)
            cancel_path()
        return False
    else:
        rospy.sleep(2)
        rospy.loginfo("Goal Reached")
        return True


def _robot_pose_callback(pose):
    """ Reads the x, y and orientation of the robot from the /amcl_pose topic
    Args:
        pose (PoseWithCovarianceStamped): estimated pose of the robot
    """
    global robot_x, robot_y, robot_orientation
    robot_x = pose.pose.pose.position.x
    robot_y = pose.pose.pose.position.y
    robot_orientation = pose.pose.pose.orientation


if __name__ == '__main__':
    try:
        pub_goal_pose(0.0,  0.0, 0.0)
        #pub_goal_pose(0.0, -6.0, 0.0)
    except rospy.ROSInterruptException:
        pass
