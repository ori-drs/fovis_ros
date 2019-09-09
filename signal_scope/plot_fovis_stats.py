import numpy
import sys
sys.argv = ['test']
import rospy
import tf

addPlot(timeWindow=10, yLimits=[-10, 10])
addSignal('/fovis/stats', msg.header.stamp, msg.status)

addPlot(timeWindow=10, yLimits=[-10, 10])
addSignal('/fovis/stats', msg.header.stamp, msg.num_inliers)

