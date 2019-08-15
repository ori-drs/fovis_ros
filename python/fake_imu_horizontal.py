#!/usr/bin/env python
# this listens to a pose message and published an imu message with the orientation only

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import sys
import math
import numpy as np

pub = rospy.Publisher('/state_estimator/fake_imu', Imu, queue_size=10)

def callback(msg):
    print "got pose, publish fake imu"
    m = Imu()
    m.header = msg.header
    m.orientation = msg.pose.pose.orientation
    pub.publish(m)

def listener():
    rospy.init_node('fake_imu', anonymous=True)
    rospy.Subscriber("/state_estimator/pose_in_odom", PoseWithCovarianceStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()