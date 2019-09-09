#!/usr/bin/env python
import rospy

import numpy as np
import csv
import time
import sys
import tf

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import *

import tf

import math
import numpy


def callBack(msg):
    print "got it"

    pos = numpy.array([0,0,0.1])
    rpy = numpy.array([0,0,3.142]) #
    quat = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])

    br.sendTransform(pos,
                     quat,
                     msg.header.stamp,
                     "/os1_lidar",
                     "/base")


br = tf.TransformBroadcaster()
rospy.init_node('nerf', anonymous=True)

rospy.Subscriber("/fovis/pose_in_odom", PoseWithCovarianceStamped, callBack)

rospy.loginfo("nerf is live")

rospy.spin()






def talker():
    pos = [2,2,-0.5]
    theta = 0

    print rospy.Time.now()

    while (1==1):
        br = tf.TransformBroadcaster()
        br.sendTransform(pos,
                         tf.transformations.quaternion_from_euler(0, 0, theta),
                         rospy.Time.now(),
                         "map",
                         "odom")
        time.sleep(0.01)
        print "x"

    #br.sendTransform(pos,
    #                 tf.transformations.quaternion_from_euler(0, 0, theta),
    #                 rospy.Time.now(),
    #                 "map",
    #                 "odom")
    #time.sleep(1)

    #br.sendTransform(pos,
    #                 tf.transformations.quaternion_from_euler(0, 0, theta),
    #                 rospy.Time.now(),
    #                 "map",
    #                 "odom")


rospy.init_node('talker', anonymous=True)
#time.sleep(0.1)

talker()
