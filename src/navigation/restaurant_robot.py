#!/usr/bin/env python
from geometry_msgs import msg
import rospy
import random
# import nav_msgs.msg Odometry
from msg import Timer

import rospy
import math
import sys
import signal
from time import time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from utils.geom import rotateQuaternion, getHeading

XY_TOLERANCE = 0.5
ORIENTATION_TOLERANCE = 0.5
MAX_TIME = 60.0  # Seconds


def signal_handler(signal, frame):
    print("\nInterrupted")
    sys.exit(1)


signal.signal(signal.SIGINT, signal_handler)


def pub_goal_pose(x, y, theta):
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

    test_pub()

    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                     _robot_pose_callback, queue_size=10)

    rospy.Subscriber("/timer", Timer, test_callback, queue_size=10)

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

        # print("diff: ", abs(robot_orientation.w - checkpoint.pose.orientation.w))
        # print("robot_orientation.w: ", robot_orientation.w)
        # print("checkpoint.pose.orientation.w: ", checkpoint.pose.orientation.w)
        # print()
        time_passed = time() - init_time
        rate.sleep()
    
    # if time_passed > MAX_TIME:
    #     rospy.logwarn("Time Ran Out")
    #     return False
    # else:
    #     rospy.sleep(2)
    #     rospy.loginfo("Goal Reached")
    #     return True


def _robot_pose_callback(pose):
    global robot_x, robot_y, robot_orientation
    robot_x = pose.pose.pose.position.x
    robot_y = pose.pose.pose.position.y
    robot_orientation = pose.pose.pose.orientation

def test_callback(msg):
    rospy.loginfo("I got a message")
    rospy.loginfo('tableID: {}, maxTime: {}'.format(msg.tableID, msg.time, msg.done))


def test_pub():
    time_pub = rospy.Publisher('/timer', Timer, queue_size=10)
    t = Timer()
    t.tableID = random.randint(0,10)
    t.time = random.randint(0,5)
    t.done = False
    time_pub.publish(t)


def main():
    
    # # rospy.Publisher("<topic name>", <message class>, <queue_size>)
    # pub = rospy.Publisher('order', Order, queue_size=10)
    # monitor = TableMonitor(pub, tables)
    pub_goal_pose(0.0, 0.0, 0.0)
    pub_goal_pose(0.0, 7.0, 0.0)
    pub_goal_pose(0.0, 0.0, 0.0)
    pub_goal_pose(0.0, -7.0, 0.0)
    pub_goal_pose(4.0, 0.0, 0.0)
    pub_goal_pose(0.0, 0.0, 0.0)
    pub_goal_pose(-4.0, 0.0, 0.0)

    # # rospy.Subscriber("<topic name>", <message class>, <callback>)
    # rospy.Subscriber("/odom", Odometry, monitor.callback)

if __name__=='__main__':
    rospy.init_node('restaurant_robot')
    # rospy.init_node('restaurant')
    rate_interval = rospy.Rate(1)
    while not rospy.is_shutdown():
        main()
        rate_interval.sleep()