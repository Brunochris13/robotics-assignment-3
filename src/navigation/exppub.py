#!/usr/bin/env python

import sys
from geometry_msgs import msg
import rospy
import math
import random
import numpy as np
from geometry_msgs.msg import (Pose, Transform, PoseWithCovarianceStamped,
                               PoseArray, Quaternion, TransformStamped)
from tf.msg import tfMessage
from tf import transformations
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from copy import deepcopy
from threading import Lock
from navigation import sensor_model 
from utils.geom import getHeading, rotateQuaternion
from navigation.histogram import Histogram
import Order


class TableMonitor(object):
    def __init__(self, pub, tables):
        self._pub = pub
        self._tables = tables
    
    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        rospy.loginfo('x: {}, y: {}'.format(x, y))
        
        # you don't need to create class object everytime, if you want you can just publish a single type
        order = tableID()
        order.tableID = random.random(x+y)
        self._pub.publish(order)