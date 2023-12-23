#! /usr/bin/env python3
# CINOVASI - HAM MAF
# Node name : lidar_filter
# Node number : 
# This node is used to limit the lidar measurement data

#-----------------------------------------------------------------------------#
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
import time
import math

RUN = False

angle=[]
ranges=[]

def callback(msg):
    global angle
    global ranges

    angle = []
    ranges = []
    #1440/2 = 720
    #print len(msg.ranges)
    # for i in range(180):
    #     if not math.isinf(msg.ranges[i]):
    #         angle.append(np.degrees(msg.angle_min+i*msg.angle_increment))
    #         ranges.append(msg.ranges[i]) 
    for i in range(len(msg.ranges)): 
        angles = np.degrees(msg.angle_min+i*msg.angle_increment)
        if angles>0 and angles<180: 
            if not math.isinf(msg.ranges[i]): 
                angle.append(angles)
                ranges.append(msg.ranges[i])
    #regions=[
        #min(msg.ranges[0:180]),
        #min(msg.ranges[60:119]),
        #min(msg.ranges[120:179])
    #    ranges, angle
    #]
    #print(a)
    #rospy.loginfo(regions)
    
    #pub_msg.angle = angle
    pub_msg.ranges = ranges
    pub_msg.intensities = angle
    pub.publish(pub_msg)

rospy.init_node('filtered_lidar')
freq = 50 # Hz
pub_msg = LaserScan()
pub = rospy.Publisher('/lidar', LaserScan, queue_size=1)
rate = rospy.Rate(freq) # Hz

rospy.Subscriber('/scan', LaserScan, callback)

print("Waiting data from LiDAR...")
while not RUN:
    RUN = True
    # RUN = RUN_gnss_front and RUN_gnss_rear
    time.sleep(0.02) # 20 ms
    pass
print("Data from LiDAR received.")
print("Laser_Scan_Data_Running")

rospy.spin()

#def main():
    #rospy.init_node('filtered_data')
    #freq = 50 # Hz    
    #pub_msg = data_final()
    #rospy.init_node('scan_values')
#    sub = rospy.Subscriber('/scan', LaserScan, callback)
    #rospy.Subscriber('/scan', LaserScan, callback)

    #pub = rospy.Publisher('/data_baru', data_final, queue_size=1)
    #rate = rospy.Rate(freq) # Hz
    #print("Waiting data from LiDAR...")
    #while not RUN:
    #    RUN = True
    #    # RUN = RUN_gnss_front and RUN_gnss_rear
    #    time.sleep(0.02) # 20 ms
    #pass
    #print("Data from LiDAR received.")
    #print("Laser_Scan_Data_Running")    
#    rospy.spin()

#if __name__=='__main__':
#    main()
