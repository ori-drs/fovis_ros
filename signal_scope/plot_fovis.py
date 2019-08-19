import numpy
import sys
sys.argv = ['test']
import rospy
import tf

# needed because no time in extractBattery
global latestT
latestT = None

def getEuler(orientation):
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w)
    return tf.transformations.euler_from_quaternion(quaternion)

def getRollDegrees(msg):
    '''roll degrees'''
    return msg.header.stamp, getEuler(msg.pose.pose.orientation)[0]*180.0/numpy.pi

def getPitchDegrees(msg):
    '''pitch degrees'''
    return msg.header.stamp, getEuler(msg.pose.pose.orientation)[1]*180.0/numpy.pi

def getYawDegrees(msg):
    '''yaw degrees'''
    return msg.header.stamp, getEuler(msg.pose.pose.orientation)[2]*180.0/numpy.pi

def getRollDegreesImu(msg):
    '''roll degrees'''
    return msg.header.stamp, getEuler(msg.orientation)[0]*180.0/numpy.pi

def getPitchDegreesImu(msg):
    '''pitch degrees'''
    return msg.header.stamp, getEuler(msg.orientation)[1]*180.0/numpy.pi

def getYawDegreesImu(msg):
    '''yaw degrees'''
    return msg.header.stamp, getEuler(msg.orientation)[2]*180.0/numpy.pi

addPlot(timeWindow=10, yLimits=[-10, 10])
#addSignal('/state_estimator/pose_in_odom', msg.header.stamp, msg.pose.pose.position.x)
addSignal('/fovis/pose_in_odom', msg.header.stamp, msg.pose.pose.position.x)
#addSignal('/vilens/pose', msg.header.stamp, msg.pose.pose.position.x)

#addSignal('/state_estimator/pose_in_odom', msg.header.stamp, msg.pose.pose.position.y)
addSignal('/fovis/pose_in_odom', msg.header.stamp, msg.pose.pose.position.y)
#addSignal('/vilens/pose', msg.header.stamp, msg.pose.pose.position.y)

#addSignal('/state_estimator/pose_in_odom', msg.header.stamp, msg.pose.pose.position.z)
addSignal('/fovis/pose_in_odom', msg.header.stamp, msg.pose.pose.position.z)
#addSignal('/vilens/pose', msg.header.stamp, msg.pose.pose.position.z)

addPlot(timeWindow=10, yLimits=[-10, 10])
#addSignalFunction('/state_estimator/pose_in_odom', getRollDegrees)
addSignalFunction('/fovis/pose_in_odom', getRollDegrees)
#addSignalFunction('/vilens/pose', getRollDegrees)
addSignalFunction('/imu_out', getRollDegreesImu)

addPlot(timeWindow=10, yLimits=[-10, 10])
#addSignalFunction('/state_estimator/pose_in_odom', getPitchDegrees)
addSignalFunction('/fovis/pose_in_odom', getPitchDegrees)
#addSignalFunction('/vilens/pose', getPitchDegrees)
addSignalFunction('/imu_out', getPitchDegreesImu)

addPlot(timeWindow=10, yLimits=[-10, 10])
#addSignalFunction('/state_estimator/pose_in_odom', getYawDegrees)
addSignalFunction('/fovis/pose_in_odom', getYawDegrees)
#addSignalFunction('/vilens/pose', getYawDegrees)
addSignalFunction('/imu_out', getYawDegreesImu)
