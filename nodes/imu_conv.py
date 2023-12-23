#! /usr/bin/env python3
# CINOVASI - HAM MAF
# Node name : data_conv
# Node number : 4
# This node is used to convert every raw data to clean data that ready to be used by another node

#-----------------------------------------------------------------------------#
# Function/library declaration
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
#-----------------------------------------------------------------------------#
# create header for publisher node
# Message publisher initialization
rospy.init_node('imu_converter')
freq = 20 # Hz
pub = rospy.Publisher('/imu_conv', PoseWithCovarianceStamped, queue_size=1)
rate = rospy.Rate(freq) # Hz
pub_msg = PoseWithCovarianceStamped()
pub_msg.header.frame_id = 'imu_converter_node'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()
#-----------------------------------------------------------------------------#
# Initialization parameter
# imu_x, imu_y, imu_z, imu_w = 0.0, 0.0, 0.0, 0.0
imu_roll, imu_pitch, imu_yaw = 0.0, 0.0, 0.0

#-----------------------------------------------------------------------------#
# other function 
def to_euler(x, y, z, w): 
    roll = np.arctan2(2.0 * (y*x + w*z), w**2 + x**2 - y**2 - z**2)
    pitch = np.arcsin(-2.0 * (x*z - w*y)/(w**2 + x**2 + y**2 + z**2))
    yaw = np.arctan2(2.0 * (w*z + x*y), w**2 - x**2 - y**2 + z**2)
    return np.array([roll, pitch, yaw])
#-----------------------------------------------------------------------------#
# Callback function -> CHECK!
def callback_imu(msg): 
    global imu_roll, imu_pitch, imu_yaw
    x = msg.orientation.x
    y = msg.orientation.y
    z = msg.orientation.z
    w = msg.orientation.w
    euler = to_euler(x,y,z,w)
    imu_roll = euler[0]
    imu_pitch = euler[1]
    imu_yaw = euler[2]           
#-----------------------------------------------------------------------------#
# subscriber initialization
rospy.Subscriber('/imu/data', Imu, callback_imu)
#-----------------------------------------------------------------------------#
while not rospy.is_shutdown():
    # list the message to publish
    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.pose.covariance[9] = np.degrees(imu_roll)
    pub_msg.pose.covariance[10] = np.degrees(imu_pitch)
    pub_msg.pose.covariance[11] = np.degrees(imu_yaw)
    # Publish the message
    pub.publish(pub_msg)

    ### Wait until the next loop
    rate.sleep()
