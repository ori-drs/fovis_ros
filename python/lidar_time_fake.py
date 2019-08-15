#!/usr/bin/env python
# needed because os1 published data with odd timestamps

import rospy

import numpy as np
import csv
import time
import sys
import tf

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import *
from sensor_msgs.msg import *

import tf

import math
import numpy

global stamp
stamp = None


def callBack(msg):
    global stamp
    print "got it"

    stamp = msg.header.stamp


def callBackCloud(msg):
    print "got cloud"
    global stamp
    msg.header.stamp = stamp
    pub.publish(msg)

rospy.init_node('nerf', anonymous=True)

pub = rospy.Publisher("/os1_cloud_node/points_fixed", PointCloud2, queue_size=10)

rospy.Subscriber("/fovis/pose_in_odom", PoseWithCovarianceStamped, callBack)

rospy.Subscriber("/os1_cloud_node/points", PointCloud2, callBackCloud)


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
