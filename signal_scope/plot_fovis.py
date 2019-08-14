import numpy
import sys
sys.argv = ['test']
import rospy
import tf

# needed because no time in extractBattery
global latestT
latestT = None

def getEuler(pose):
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    return tf.transformations.euler_from_quaternion(quaternion)

def getRollDegrees(msg):
    '''roll degrees'''
    return msg.header.stamp, getEuler(msg.pose.pose)[0]*180.0/numpy.pi

def getPitchDegrees(msg):
    '''pitch degrees'''
    return msg.header.stamp, getEuler(msg.pose.pose)[1]*180.0/numpy.pi

def getYawDegrees(msg):
    '''yaw degrees'''
    return msg.header.stamp, getEuler(msg.pose.pose)[2]*180.0/numpy.pi

addPlot(timeWindow=10, yLimits=[-10, 10])
addSignal('/state_estimator/pose_in_odom', msg.header.stamp, msg.pose.pose.position.x)
addSignal('/fovis/pose_in_odom', msg.header.stamp, msg.pose.pose.position.x)

addSignal('/state_estimator/pose_in_odom', msg.header.stamp, msg.pose.pose.position.y)
addSignal('/fovis/pose_in_odom', msg.header.stamp, msg.pose.pose.position.y)

addSignal('/state_estimator/pose_in_odom', msg.header.stamp, msg.pose.pose.position.z)
addSignal('/fovis/pose_in_odom', msg.header.stamp, msg.pose.pose.position.z)

addPlot(timeWindow=10, yLimits=[-10, 10])
addSignalFunction('/state_estimator/pose_in_odom', getRollDegrees)
addSignalFunction('/fovis/pose_in_odom', getRollDegrees)

addPlot(timeWindow=10, yLimits=[-10, 10])
addSignalFunction('/state_estimator/pose_in_odom', getPitchDegrees)
addSignalFunction('/fovis/pose_in_odom', getPitchDegrees)

addPlot(timeWindow=10, yLimits=[-10, 10])
addSignalFunction('/state_estimator/pose_in_odom', getYawDegrees)
addSignalFunction('/fovis/pose_in_odom', getYawDegrees)