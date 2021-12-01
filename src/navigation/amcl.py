#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Quaternion
from tf.msg import tfMessage
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan


def main():
    # Publishers
    amcl_pose_publisher = rospy.Publisher(
        "/amcl_pose", PoseWithCovarianceStamped)
    particle_cloud_publisher = rospy.Publisher("/particlecloud", PoseArray)
    tf_publisher = rospy.Publisher("/tf", tfMessage)

    # Subscribers
    laser_subscriber = rospy.Subscriber("/base_scan", LaserScan,
                                              laser_callback,
                                              queue_size=1)
    initial_pose_subscriber = rospy.Subscriber("/initialpose",
                                                     PoseWithCovarianceStamped,
                                                     initial_pose_callback)
    odometry_subscriber = rospy.Subscriber("/odom", Odometry,
                                                 odometry_callback,
                                                 queue_size=1)

def laser_callback():
    pass

def initial_pose_callback():
    pass

def odometry_callback():
    pass


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
