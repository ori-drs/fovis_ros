#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import numpy as np
import csv

results =[]
with open('example_trajectory.csv', 'rb') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in spamreader:
        # skip lines beginning with #
        if row[0][0] == "#":
            continue

        result = [float(i) for i in row]
        #print result
        results.append(result)

#print results

def talker():
    pub = rospy.Publisher('/trajectory', Path, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    path_msg = Path()
    for result in results:
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        # todo - do proper conversion to timestamp
        rem = (result[0] % 1)*1e9
        pose.header.stamp.secs = np.round(result[0])
        pose.header.stamp.nsecs = rem

        pose.pose.position.x=result[1]
        pose.pose.position.y=result[2]
        pose.pose.position.z=result[3]
        pose.pose.orientation.w=result[4]
        pose.pose.orientation.x=result[5]
        pose.pose.orientation.y=result[6]
        pose.pose.orientation.z=result[7]
        path_msg.poses.append(pose)

    path_msg.header.frame_id = "odom"

    while not rospy.is_shutdown():
        hello_str = "publish path %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass