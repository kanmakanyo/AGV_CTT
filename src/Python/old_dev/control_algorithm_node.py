#!/usr/bin/env python3
# CINOVASI - HAM MAF
# this node is intended to calculate the navigation signal to the low level controller
# this node subscribes the sensor data value and calculate the navigation signal based on the error and 
#   control algorithm used
# the control algorithm is placed on the control_algorithm_func.py file
# 
#-----------------------------------------------------------------------------#
# library declaration
from statistics import mode
import rospy
import time
import sys
import os
import numpy as np
# from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
# from stanley_2d import Controller --> replace with class from control_algorithm_func.py
from control_algorithm_func import Controller
# from lyapunov_rl import Controller
#-----------------------------------------------------------------------------#
# Initialization Parameters
# beta = 0.2
cs_lat_last = 0
lever_mode = 0              #0: neutral; 1: forward; 2: reverse
lift_ena = 0                #0: off; 1: lift up; 2: lift down
zc6_now = 0
zc7_now = 0
zc6_target = 0
zc7_target = 0
counter_steer_command = 0   #give delay for steering command
count = 0                   #for testing purpose
control_mode = 0            #initial control mode
steer_angle_bef = 0         #initial steering angle (for steer check only), in rad
steer_now = 0               #initial actual steering angle, in rad
cs_lat = 0 
cs_long = 0
cs_brake = 0
cs_lamp = 0
cs_lat_bef = 0
time_base = time.time()
REVEIVED_STATE_VAL = False
k_rho = rospy.get_param('~k_rho', 0.1)
k_alpha = rospy.get_param('~k_alpha', 0.05)
kv_alpha = rospy.get_param('~kv_alpha', 50)
ks = rospy.get_param('~ks', 5)
kv = rospy.get_param('~kv', 1)
sat_lat_max = rospy.get_param('~sat_lat_max', 0.785)
sat_lat_min = rospy.get_param('~sat_lat_min', -0.785)
sat_long_max = rospy.get_param('~sat_long_max', 1)
stop_limit = rospy.get_param('~stop_limit', 3)
waypoints_path = rospy.get_param('~waypoints_path', 'sigmoid200722.npy')
waypoints_path = os.path.abspath(sys.path[0] + '/../src/waypoints/' + waypoints_path)
PSback_X = rospy.get_param('~PSback_X', 558091.836)#558092.34)
PSback_Y = rospy.get_param('~PSback_Y', 9157947.658)#9157962.32)
PSback_yaw = rospy.get_param('~PSback_yaw', 1.57596)
length = rospy.get_param('~length', 14.02)#13.75)
dock_X = rospy.get_param('~dock_X', 558091.836)#558092.34)
dock_Y = rospy.get_param('~dock_Y', 9157929.288)#9157928.938)#9157952.32)
dock_yaw = rospy.get_param('~dock_yaw', 1.57596)
print("waypoints file:",waypoints_path)

sat_lat = np.array([sat_lat_min, sat_lat_max])
waypoints = np.load(waypoints_path)
#-----------------------------------------------------------------------------#
# While: pemanggilan class
# Class yang dipanggil disesuaikan dengan mode
# Data yang dipulish: 
# command sudut steer
# command speed
# command autonomous mode ke HMI
#-----------------------------------------------------------------------------#
# Message publisher initialization
rospy.init_node('control_algorithm')
freq = 20 # Hz
pub = rospy.Publisher('/control_algorithm', PoseWithCovarianceStamped, queue_size=1)
rate = rospy.Rate(freq) # Hz
pub_msg = PoseWithCovarianceStamped()
pub_msg.header.frame_id = 'agv_control_algorithm_node'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()
# last_time = pub_msg.header.stamp.to_sec() - 1./freq
# last_time = rospy.Time.now().to_sec() - 1./freq
#-----------------------------------------------------------------------------#
# Callback Function
state = {'x': 0., 'y': 0., 'yaw': 0., 'v': 0., 'steer_now': 0.}
def callback_Data_Conv(msg): 
    global RECEIVED_STATE_VAL, steer_now, v
    global state
    state['x'] = msg.pose.covariance[4] #x pos
    state['y'] = msg.pose.covariance[5] #y pos
    state['yaw'] = msg.pose.covariance[7] #yaw now
    state['v'] = msg.pose.covariance[6] #v now
    state['steer_now'] = msg.pose.covariance[2] #in radian
    RECEIVED_STATE_VAL = True   

def callback_HMI(msg):
    global control_mode, zc6_target, zc7_target
    control_mode = msg.pose.covariance[0]
    if control_mode==6: 
        #lift up!
        zc6_target = 5211#5120
        zc7_target = 5280#5180
    elif control_mode==7:
        #lift mid!
        zc6_target = 4540 + 124
        zc7_target = 4578 + 157
        if zc6_now> zc6_target: 
            zc6_target = 4540 + 424
            zc7_target = 4578 + 457
    elif control_mode==8:
        #lift mid!
        zc6_target = 4540
        zc7_target = 4578
        # if zc6_now> zc6_target: 
        #     zc6_target = 4540 + 300
        #     zc7_target = 4578 + 300

# Callback function
def callback_dbwsub(msg):
    # Get data adc from PLC (ADC) --> Feedback (sensor)
    global zc6_now, zc7_now
    zc6_now = msg.pose.covariance[5]
    zc7_now = msg.pose.covariance[3]

# def callback_state(msg):
#     # Get data adc from PLC (ADC) --> Feedback (sensor)
#     global RECEIVED_STATE_VAL
#     global state
#     state['x'] = msg.pose.covariance[4] #x pos
#     state['y'] = msg.pose.covariance[5] #y pos
#     state['yaw'] = msg.pose.covariance[7] #yaw now
#     state['v'] = msg.pose.covariance[6] #v now
#     # state['steer_now'] = msg.pose.covariance[2] #in radian
#     RECEIVED_STATE_VAL = True 

#-----------------------------------------------------------------------------#
# While: pemanggilan class
# Class yang dipanggil disesuaikan dengan mode
#-----------------------------------------------------------------------------#
controller = Controller(k_rho, k_alpha, kv_alpha, ks, kv, sat_lat, sat_long_max,\
                 waypoints, stop_limit, PSback_X, PSback_Y, PSback_yaw, length,\
                 dock_X, dock_Y, dock_yaw)                 
#-----------------------------------------------------------------------------#
# Konsep: 
# - subscribe data posisi, orientasi, sudut setir, speed
# - subscribe mode dari HMI 
# 0 Stand by
# 1 Steering Test
# 2 Propulsion Test
# 3 Path Tracking
# 4 Point Stabilization
# 5 Docking
# Message subscriber initialization
rospy.Subscriber('/data_conv', PoseWithCovarianceStamped, callback_Data_Conv)
rospy.Subscriber('/dbw_pubsub', PoseWithCovarianceStamped, callback_dbwsub)
rospy.Subscriber('/idc_lamp', PoseWithCovarianceStamped, callback_HMI)
# rospy.Subscriber('/state_position', PoseWithCovarianceStamped, callback_state)
#-----------------------------------------------------------------------------#
while not rospy.is_shutdown():
    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    ### Calculate the actual sampling time
    # delta_t = pub_msg.header.stamp.to_sec() - last_time
    # last_time = pub_msg.header.stamp.to_sec()

    if control_mode==0: 
        #DO: nothing, send the command to keep the straight steering angle and zero speed
        count = 0 #for testing purpose
        cs_lat = 0 
        cs_long = 0
        cs_brake = 0
        cs_lamp = 0
        steer_angle_bef = 0
        lever_mode = 0
    
    elif control_mode==1: 
        #DO: give movement check action to the steering angle (move right-left-straight)
        # print("Mode 1 Received! Do steering check")
        if steer_angle_bef < 0.052 and steer_angle_bef > -0.052: #between 3 and -3 degrees
        # if count < 100:
            # in this condition, move the steering angle to the right 20 degrees
            lever_mode = 1 #set forward movement
            cs_lat = -0.349
            cs_lamp = 1
            # print("Move to the right!")
            if state['steer_now'] < -0.331 and state['steer_now'] > -0.366: #between -19 and -21 degrees
            # if count >100 and count < 102:
                # print("right angle achieved! wait for 1 second")
                lever_mode = 1 #set forward movement
                steer_angle_bef = -0.349
                time.sleep(1) #berhenti selama 3 detik
        elif steer_angle_bef < -0.331 and steer_angle_bef > -0.366: #between -19 and -21 degrees
        # elif count >102 and count < 200: 
            # in this condition, move the steering angle to the left 20 degrees
            lever_mode = 1 #set forward movement
            cs_lat = 0.349
            cs_lamp = 1
            # print("move to the left!")
            if state['steer_now'] < 0.366 and state['steer_now'] > 0.331: #between 19 and 21 degrees
            # if count >200 and count < 202:
                # print("left angle achieved! wait for 1 second")
                lever_mode = 1 #set forward movement
                steer_angle_bef = 0.349
                time.sleep(1)
        elif steer_angle_bef < 0.366 and steer_angle_bef > 0.331: #between 19 and 21 degrees
        # elif count > 202:
            lever_mode = 1 #set forward movement
            cs_lat = 0
            cs_lamp = 1
            # print("go back to straight heading!")
            if state['steer_now'] < 0.052 and state['steer_now'] > -0.052: #between 3 and -3 degrees
            # if count>300:
                # control_mode = 0
                cs_lamp = 0  #give finish signal to HMI
                lever_mode = 0 #set neutral
                # print("finished! back to stand by mode in 1 second")
                time.sleep(1)
        # publish the message
        cs_long = 0
        cs_brake = 0
        count +=1 
    
    elif control_mode == 2: 
        #DO: give propulsion command for 2 second 
        if time.time() - time_base > 25: #renew the time_base value
            time_base = time.time()
            # print("reset the time_base value!")
        if time.time() - time_base < 8: #move forward 3 sec
            lever_mode = 1 #set forward movement
            cs_long = 1 #move 1 m/s
            cs_lamp = 1
            # print("move for 2 second with 1 m/s speed")
        elif time.time() - time_base > 8 and time.time() - time_base < 13: #stop for 2 sec 
            lever_mode = 0 #set neutral and stop
            cs_long = 0 
            cs_lamp = 1
            # control_mode = 0
            # print("finished! get back to standby mode")
        elif time.time() - time_base > 13 and time.time() - time_base < 20: #move backward 3 sec
            lever_mode = 2 #set reverse
            cs_long = 1
            cs_lamp = 1
        elif time.time() - time_base > 20 : #finish
            lever_mode = 0 #set neutral and stop
            cs_long = 0
            cs_lamp = 0
            control_mode = 0
            
        # publish the message
        cs_lat = 0
        cs_brake = 0

    elif control_mode == 3: 
        #DO: path following control with motion planning sigmoid curve
        # important! make sure the vehicle doesnt have large lateral value with the waypoints
        # important! make sure the movement mode has been changed to forward mode
        # will work only if the state variable (/Data_Conv) topic is available
        if (RECEIVED_STATE_VAL):
            lever_mode = 1 #set forward movement
            # debug message
            # print("Autonomous Path Following Mode!")
            ### Calculate the control signal
            xpath = state['x'] + 11.92 * np.cos(state['yaw'])
            ypath = state['y'] + 11.92 * np.sin(state['yaw'])
            # finish_flag, cs_long, cs_lat, cs_brake = controller.calculate_path_following_signal(state['x'],
            #                                                 state['y'], state['v'],
            #                                                 state['yaw'])
            finish_flag, cs_long, cs_lat, cs_brake = controller.calculate_path_following_signal(xpath,
                                                            ypath, state['v'],
                                                            state['yaw'])
            cs_lamp = 1
            if finish_flag == 1:
            # if count > 200: 
                # print("Finished! Get back to stand by mode")
                # control_mode = 0 
                lever_mode = 0 #set neutral and stop
                cs_lamp = 0  
                cs_lat = 0
                cs_long = 0
                cs_brake = 0
        else:
            print("[INFO] [", rospy.Time.now(), "] State Variable has not received yet. Cannot start autonomous mode.")
        
        count +=1
    
    elif control_mode == 4: 
        #DO: backward motion with point stabilization control algorithm
        # important! make sure the movement mode is set to backward
        # will work only if the state variable (/Data_Conv) topic is available
        if (RECEIVED_STATE_VAL):
            # debug message
            # print("Autonomous Point Stabilization Mode!")
            lever_mode = 2 #set reverse movement
            ### Calculate the control signal
            finish_flag, cs_long, cs_lat, cs_brake = controller.calculate_point_stab_signal(state['x'],
                                                            state['y'], state['v'],
                                                            state['yaw'])
            # if controller.errx_ < 1:
            #     cs_lat = beta * cs_lat_last + (1-beta)*cs_lat
            # else:
            #     cs_lat_last = cs_lat
            cs_lamp = 1
            if finish_flag == 1: 
            # if count>200:
                # print("Finished! Get back to stand by mode")
                lever_mode = 0 #set neutral and stop
                # control_mode = 0 
                cs_lamp = 0  
                cs_lat = 0
                cs_long = 0
                cs_brake = 0
        else:
            print("[INFO] [", rospy.Time.now(), "] State Variable has not received yet. Cannot start autonomous mode.")
        
        count +=1
    
    elif control_mode == 5: 
        #DO: backward motion with point stabilization control algorithm
        # important! make sure the movement mode is set to backward
        # will work only if the state variable (/Data_Conv) topic is available
        if (RECEIVED_STATE_VAL):
            # debug message
            # print("Autonomous Docking Mode!")
            lever_mode = 2 #set reverse movement
            ### Calculate the control signal
            finish_flag, cs_long, cs_lat, cs_brake = controller.calculate_docking_signal(state['x'],
                                                            state['y'], state['v'],
                                                            state['yaw'])
            cs_lamp = 1
            if finish_flag == 1: 
            # if count > 200:
                # print("Finished! Get back to stand by mode")
                lever_mode = 0 #set neutral and stop
                control_mode = 0 
                cs_lamp = 0  
                cs_lat = 0
                cs_long = 0
                cs_brake = 0
        else:
            print("[INFO] [", rospy.Time.now(), "] State Variable has not received yet. Cannot start autonomous mode.")
        
        count += 1
    
    elif control_mode == 6: 
        #DO: lifting up!
        # will work only if the state variable (/Data_Conv) topic is available
        # zc6_target = 5100
        # zc7_target = 5080
        if zc6_now>zc6_target-10 and zc7_now>zc7_target-10: #20 ADC tolerance
        #Do: stop lifting process
            lift_ena = 0
            control_mode = 0
            cs_lamp = 0

        elif zc6_now < zc6_target and zc7_now < zc6_target: 
            #Do: lift up!
            lift_ena = 1
            cs_lamp = 1

        # publish the message
        cs_lat = 0
        cs_long = 0
        cs_brake = 0
        lever_mode = 0

    elif control_mode == 7: 
        #DO: lifting mid!
        # will work only if the state variable (/Data_Conv) topic is available
        if zc6_now>5100 or zc7_now>5100: 
            zc6_target = 4990 + 124
            zc7_target = 5035  + 157
        # elif zc6_now<5200 and zc7_now<5200 and cs_lamp == 0:
        #     zc6_target = 4540 + 124
        #     zc7_target = 4578 + 157

        if zc6_now>zc6_target-50 and zc7_now>zc7_target-50 and zc6_now<zc6_target+50 and zc7_now<zc7_target+50: #20 ADC tolerance
        #Do: stop lifting process
            lift_ena = 0
            control_mode = 0
            cs_lamp = 0
        elif zc6_now < zc6_target and zc7_now < zc6_target: 
            #Do: lift up!
            lift_ena = 1
            cs_lamp = 1
        elif zc6_now > zc6_target and zc7_now > zc6_target: 
            #Do: lift down!
            lift_ena = 2
            cs_lamp = 1

        # publish the message
        cs_lat = 0
        cs_long = 0
        cs_brake = 0
        lever_mode = 0
    
    elif control_mode == 8: 
        #DO: lifting down!
        # will work only if the state variable (/Data_Conv) topic is available
        if zc6_now>5000 or zc7_now>5000:
            zc6_target = 4990
            zc7_target = 5035
        # elif zc6_now<5200 and zc7_now<5200 and cs_lamp == 0:
        #     zc6_target = 4540
        #     zc7_target = 4578

        if zc6_now<zc6_target+70 and zc7_now<zc7_target+40: #20 ADC tolerance
        #Do: stop lifting process
            lift_ena = 0
            control_mode = 0
            cs_lamp = 0
        elif zc6_now > zc6_target and zc7_now > zc6_target: 
            #Do: lift up!
            lift_ena = 2
            cs_lamp = 1

        # publish the message
        cs_lat = 0
        cs_long = 0
        cs_brake = 0
        lever_mode = 0

    # if counter_steer_command == 0: #to give delay for steering command
    #     # DO: update cs_lat before
    #     cs_lat_bef = cs_lat
    # else: 
    #     # DO: give cs_lat before as control signal
    #     cs_lat = cs_lat_bef

    # # update and reset the counter
    # counter_steer_command += 1
    # if counter_steer_command > 4:
    #     counter_steer_command = 0
    
    # list the message to publish
    pub_msg.pose.covariance[0] = cs_lat #steering angle in rad
    pub_msg.pose.covariance[1] = cs_long #speed set point in m/s
    pub_msg.pose.covariance[2] = cs_brake #braking percentage
    pub_msg.pose.covariance[3] = cs_lamp #autonomous lamp
    pub_msg.pose.covariance[4] = control_mode #debug the control mode
    pub_msg.pose.covariance[5] = state['x']
    pub_msg.pose.covariance[6] = state['y']
    pub_msg.pose.covariance[7] = state['v']
    pub_msg.pose.covariance[8] = state['yaw']
    # log the controller data
    pub_msg.pose.covariance[9] = controller._e_yaw
    pub_msg.pose.covariance[10] = controller._e_lat
    pub_msg.pose.covariance[11] = controller.rho_
    pub_msg.pose.covariance[12] = controller.errx_
    pub_msg.pose.covariance[13] = controller.erry_
    pub_msg.pose.covariance[14] = controller.vi_
    pub_msg.pose.covariance[15] = controller.alpha_
    pub_msg.pose.covariance[16] = controller.omega
    pub_msg.pose.covariance[17] = controller.erryaw_
    pub_msg.pose.covariance[18] = lever_mode
    pub_msg.pose.covariance[19] = controller.ps_cond
    pub_msg.pose.covariance[20] = lift_ena
    pub_msg.pose.covariance[21] = zc6_target
    pub_msg.pose.covariance[22] = zc7_target
    # print("errx, erry, e_yaw (rad), e_yaw (deg), cs_lat, cs_long")
    print(controller.errx_, controller.erry_, controller.erryaw_, np.degrees(controller.erryaw_), cs_lat, cs_long)

    # Publish the message
    pub.publish(pub_msg)

    ### Wait until the next loop
    rate.sleep()