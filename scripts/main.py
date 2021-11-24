import rospy

def looper(callback, freq=1):
    interval = rospy.Rate(freq)

    while not rospy.is_shutdown():
        callback()
        interval.sleep()