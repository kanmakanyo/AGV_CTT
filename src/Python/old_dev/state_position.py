#!/usr/bin/env python3

import rospy
import time
import sys
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

rad_steer = 0.
reference_voltage = 20
control_mode = 0

sensor_data = {
    'x_front': 0.,
    'y_front': 0., 
    'x_rear': 0., 
    'y_rear': 0.
}

temp_msg = {
    'x_est': 0.,
    'y_est': 0.,
    'v_est': 0.,
    'yaw_est': 0.,
    'yaw_gnss_fr': 0.,
    'x_temp': 0.,
    'y_temp': 0.,
    'yaw_trailer': 0.,
    'yaw_head': 0.
}

RUN = False
RUN_gnss_front = False
RUN_gnss_rear = False
seq_front = 0
seq_rear = 0

def callback_control(msg):
    # Get data from control_algorithmm node
    global control_mode, lever_mode 
    control_mode = msg.pose.covariance[4]
    lever_mode = msg.pose.covariance[18]

def callback_dbwsub(msg):
    # Get data adc from PLC (ADC) --> Feedback (sensor)
    global volt_f_steer, rad_steer, deg_steer, temp_msg
    autonomous_switch = msg.pose.covariance[16]
    cal_steer = msg.pose.covariance[1]      # Steer 
    # Conversi data from PLC (ADC) to volt
    # re-check the voltage value and the offset value
    volt_f_steer = ((cal_steer / 65535)* reference_voltage) #in volt (CHECK!)
    deg_steer = ((volt_f_steer - 2.6603)/0.013) + 2 #in degree
    # if control_mode == 1:
    #     deg_steer = deg_steer + 1
    YS31 = msg.pose.covariance[18]
    YS32 = msg.pose.covariance[19]
    if YS31 and not YS32: #reverse
        deg_steer = deg_steer - 3.8
    if control_mode == 1:
        deg_steer = deg_steer + 2
    elif control_mode == 4: 
        deg_steer += 2
    # if lever_mode == 2:
    #     deg_steer = deg_steer - 3.8
    # if autonomous_switch: 
    #     deg_steer = deg_steer - 9.3
    rad_steer = np.radians(deg_steer) #in radians
    # temp_msg['v_est'] = msg.pose.covariance[27]
    # print("here")

def gnssFrontCallback(msg):
    global sensor_data, seq_front
    global temp_msg
    global RUN_gnss_front

    RUN_gnss_front = True
    seq_front+= 1

    sensor_data['x_front'] = msg.pose.pose.position.x
    sensor_data['y_front'] = msg.pose.pose.position.y

    # calculate estimation speed by using gnss
    delta_t = 0.2 #ms (5 Hz)
    temp_msg['v_est'] = np.sqrt((temp_msg['x_temp']-sensor_data['x_front'])**2 + (temp_msg['y_temp']-sensor_data['y_front'])**2) / delta_t
    temp_msg['x_temp'] = sensor_data['x_front']
    temp_msg['y_temp'] = sensor_data['y_front']
    # temp_msg['yaw_gnss_fr'] = np.arctan2(sensor_data['y_front']-sensor_data['y_rear'],
    #                                                 sensor_data['x_front']-sensor_data['x_rear'])
    # temp_msg['yaw_trailer'] = temp_msg['yaw_gnss_fr'] + 1.6491#+ np.pi/2
    # temp_msg['yaw_head'] = temp_msg['yaw_trailer'] + rad_steer

    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.pose.covariance[6] = temp_msg['v_est']
    # pub_msg.pose.covariance[7] = temp_msg['yaw_trailer']
    # pub_msg.pose.covariance[8] = temp_msg['yaw_head']
    # pub_msg.pose.covariance[12] = temp_msg['yaw_gnss_fr']
    # pub_msg.yaw_gnss_fr = temp_msg['yaw_gnss_fr']
    # pub_msg.x_est = temp_msg['x_est']
    # pub_msg.y_est = temp_msg['y_est']
    # pub_msg.v_est = temp_msg['v_est']

    pub.publish(pub_msg)
    if seq_front > 100: 
        seq_front = 0
    
def gnssRearCallback(msg):
    global sensor_data, seq_rear
    global temp_msg
    global RUN_gnss_rear

    RUN_gnss_rear = True
    seq_rear +=1

    sensor_data['x_rear'] = msg.pose.pose.position.x
    sensor_data['y_rear'] = msg.pose.pose.position.y

    if seq_front == seq_rear: 
        temp_msg['yaw_gnss_fr'] = np.arctan2(sensor_data['y_front']-sensor_data['y_rear'],
                                                        sensor_data['x_front']-sensor_data['x_rear'])
        temp_msg['yaw_trailer'] = temp_msg['yaw_gnss_fr'] + 1.6491#+ np.pi/2
        temp_msg['yaw_head'] = temp_msg['yaw_trailer'] + rad_steer
    
    # trailer position calculation here!
    # from HAM notes
    # coordinates transformation
    X_cgnss = sensor_data['x_rear'] + 0.54 * np.cos(temp_msg['yaw_gnss_fr'])
    Y_cgnss = sensor_data['y_rear'] + 0.54 * np.sin(temp_msg['yaw_gnss_fr'])
    X_cfw = X_cgnss + 0.7 * np.cos(temp_msg['yaw_trailer']-(np.pi))
    Y_cfw = Y_cgnss + 0.7 * np.sin(temp_msg['yaw_trailer']-(np.pi))
    X_crw = X_cfw + 11.92 * np.cos(temp_msg['yaw_trailer'] - np.pi)
    Y_crw = Y_cfw + 11.92 * np.sin(temp_msg['yaw_trailer'] - np.pi)
    temp_msg['x_est'] = X_crw
    temp_msg['y_est'] = Y_crw

    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.pose.covariance[4] = temp_msg['x_est'] #sensor_data['x_rear'] #temp_msg['x_est']
    pub_msg.pose.covariance[5] = temp_msg['y_est'] #sensor_data['y_rear'] 
    pub_msg.pose.covariance[7] = temp_msg['yaw_trailer']
    pub_msg.pose.covariance[8] = temp_msg['yaw_head']
    pub_msg.pose.covariance[12] = temp_msg['yaw_gnss_fr']
    pub.publish(pub_msg)
    if seq_rear > 100: 
        seq_rear = 0

rospy.init_node('state_position')
freq = 5 # Hz

pub_msg = PoseWithCovarianceStamped()
pub_msg.header.frame_id = 'state_position'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()

rospy.Subscriber('/control_algorithm', PoseWithCovarianceStamped, callback_control)
rospy.Subscriber('/dbw_pubsub', PoseWithCovarianceStamped, callback_dbwsub)
rospy.Subscriber('/utm', Odometry, gnssFrontCallback)
rospy.Subscriber('/utm2', Odometry, gnssRearCallback)
pub = rospy.Publisher('/state_position', PoseWithCovarianceStamped, queue_size=1)
rate = rospy.Rate(freq) # Hz

print("Waiting data from GNSS...")
while not RUN:
    RUN = RUN_gnss_front and RUN_gnss_rear
    time.sleep(0.02) # 20 ms
    pass

print("Data from GNSS received.")
print("State estimator program is now running")
rospy.spin()
