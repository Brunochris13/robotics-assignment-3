#!/usr/bin/env python

import rospy
from agent.robot import Robot
from environment.restaurant import Restaurant


def looper(callback, freq=1):
    interval = rospy.Rate(freq)

    while not rospy.is_shutdown():
        callback()
        interval.sleep()


if __name__ == "__main__":
    restaurant = Restaurant()
    robot = Robot(restaurant)

    looper(robot.update)