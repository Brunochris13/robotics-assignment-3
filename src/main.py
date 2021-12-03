#!/usr/bin/env python

import rospy
from agent.robot import Robot

def looper(callback, freq=1):
    interval = rospy.Rate(freq)

    while not rospy.is_shutdown():
        print('hi')
        callback()
        interval.sleep()

if __name__ == "__main__":
    robot = Robot()
    looper(robot.update())