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
from control_algorithm_func_new import Controller
# from lyapunov_rl import Controller
#-----------------------------------------------------------------------------#
# Initialization Parameters
# beta = 0.2
stage = 0                   #0 for first PS, 1 for the next PS
stage_flag = 0              #0 for first PS, 1 for the next PS
filter_flag = 0 
finish_flag = 0
cs_brake = 0                #0: release, 1: 20 b, 2: 30 b
lift_flag = 0               #0: lift inactive, 1: lift active
lifting_mode = 0            #0: down, 1: mid, 2: up
cs_lat_last = 0
lever_mode = 0              #0: neutral; 1: forward; 2: reverse
lift_ena = 0                #0: off; 1: lift up; 2: lift down
zc6_now = 0
zc7_now = 0
zc8_now = 0
zc6_target = 0
zc6_target_bef = 0
zc7_target = 0
counter_steer_command = 0   #give delay for steering command
count = 0                   #for testing purpose
control_mode = 0            #initial control mode
steer_angle_bef = 0         #initial steering angle (for steer check only), in rad
steer_now = 0               #initial actual steering angle, in rad
cs_lat = 0 
cs_long = 0
# cs_brake = 0
cs_lamp = 0
cs_lat_bef = 0
time_base = time.time()
REVEIVED_STATE_VAL = False
k_rho = rospy.get_param('~k_rho', 7) # 0.1 asalanya
k_alpha = rospy.get_param('~k_alpha', 0.05) #0.05 asalnya
kv_alpha = rospy.get_param('~kv_alpha', 50)
ks = rospy.get_param('~ks', 5)
kv = rospy.get_param('~kv', 1)
sat_lat_max = rospy.get_param('~sat_lat_max', 0.785)
sat_lat_min = rospy.get_param('~sat_lat_min', -0.785)
sat_long_max = rospy.get_param('~sat_long_max', 1.5) #asalnya 1.5. DIganti pd tanggal 18 okt
stop_limit = rospy.get_param('~stop_limit', 3) #asalnya 3 diganti 18 okt
waypoints_path = rospy.get_param('~waypoints_path', 'sigmoid220722.npy')
waypoints_path = os.path.abspath(sys.path[0] + '/../src/waypoints/' + waypoints_path)
PSback_X = rospy.get_param('~PSback_X', 685552.00531)#558091.836)
PSback_Y = rospy.get_param('~PSback_Y', 9204398.54478)#9157962.32)
PSback_yaw = rospy.get_param('~PSback_yaw', 0.773214)
length = rospy.get_param('~length', 14.02)#13.75)
# GNSS
# PSback_X = 685519.7991
# PSback_Y = 9204438.239
# PSback_yaw = 0.81982
# LiDAR
PSback_X = -0.14  
PSback_Y = 19.5
PSback_yaw = np.radians(90.5)

#LiDAR
#line 2 = -15.05
#line 4 = -7.40
# x1, y1, yaw1 = -7.63, 23.5, np.radians(90.5)
# x2, y2, yaw2 = -7.63, 19.5, np.radians(90.5)
# ambil titik saat docking
# Data titik mulai docking
# x = -0.015, y=18.91 tetha = 90.05
# x = -0.024 y =21.065 tetha = 90.01
#yang udah berhasil 19 okt 2023 : 
#x1, y1, yaw1 = 0.042, 25.16, np.radians(90.34)
#x2, y2, yaw2 = 0.042, 21.16, np.radians(90.34)
#yang udah berhasil 19-20 okt 2023 :
# x1, y1, yaw1 = 0.023, 21.91, np.radians(90.05)
# x2, y2, yaw2 = 0.023, 18.91, np.radians(90.05)
#yang udah berhasil 8 des 2023 :
#x1, y1, yaw1 = 0.023, 21.91, np.radians(90.58)
#x2, y2, yaw2 = 0.023, 18.91, np.radians(90.58)
#Setting 22 Desember 2023
x1, y1, yaw1 = -0.035, 21.736, np.radians(90.53)
x2, y2, yaw2 = -0.035, 18.736, np.radians(90.53)  

#GNSS
# x1, y1, yaw1 = 685522.751, 9204441.455, 0.81982
# x2, y2, yaw2 = 685519.799, 9204438.239, 0.81982

# dock_X = rospy.get_param('~dock_X', 685507.081)#558092.836)
# dock_Y = rospy.get_param('~dock_Y', 9204424.560)#9157928.938)#9157952.32)
# dock_yaw = rospy.get_param('~dock_yaw', 0.81982)
#GNSS
# dock_X = 685507.081
# dock_Y = 9204424.560
# dock_yaw = 0.81982
#LIDAR
# ambil titik saat docking (titik belakang)

#Titik akhir berhasil 5 percobaan dari6 6 ---> 20 okt 2023
# dock_X = 0.048
# dock_Y = 1.475
# dock_yaw = np.radians(90.2)

#Titik 7 Desember 2023
#dock_X = 0.048 #Di sopas -0.081
#dock_Y = 1.475 #Di sopas 1.418
#dock_yaw = np.radians(90.55)
#Titik 22 Desember 2023
dock_X = -0.012 #Di sopas -0.012
dock_Y = 1.612 #Di sopas 1.612
dock_yaw = np.radians(90.35) #Di sopas 90.35

# undock_X = rospy.get_param('~undock_X', 685555.1928423041)#558092.34)
# undock_Y = rospy.get_param('~undock_Y', 9204399.226095065)#9157928.938)#9157952.32)
# undock_yaw = rospy.get_param('~undock_yaw', 0.7943453892182503)
#GNSS
#LiDAR
#undock_X = 0.015
#undock_Y = 16.91
#undock_yaw = np.radians(90.25)

#22des2023
undock_X = 0.035
undock_Y = 16.91
undock_yaw = np.radians(90.5)
# dock_X, dock_Y, dock_yaw
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
        zc6_target = 5003#5120
        zc7_target = 5114#5180
    elif control_mode==7:
        #lift mid!
        zc6_target = 4690 + 124#4540 + 124
        zc7_target = 4617 + 157#4578 + 157
        # if zc6_now> zc6_target: 
        #     zc6_target = 4540 + 424
        #     zc7_target = 4578 + 457
    elif control_mode==8:
        #lift down!
        zc6_target = 4624#4540
        zc7_target = 4666#4578
        # if zc6_now> zc6_target: 
        #     zc6_target = 4540 + 300
        #     zc7_target = 4578 + 300

# Callback function
def callback_dbwsub(msg):
    # Get data adc from PLC (ADC) --> Feedback (sensor)
    global zc6_now, zc7_now, zc8_now
    zc6_now = msg.pose.covariance[5]
    zc7_now = msg.pose.covariance[3]
    zc8_now = msg.pose.covariance[4]

def lifting_command(lifting_mode): 
    global lift_flag, zc6_target_bef, zc6_target, zc7_target
    global lift_ena
    if lifting_mode == 0: #down lift
        zc6_target = 4690#4540
        zc7_target = 4630#4578
        if lift_flag: #go lifting
            if zc6_now>5000 or zc7_now>5000:
                zc6_target = 4800
                zc7_target = 4800
            zc6_target_bef = zc6_target

            if zc6_now<zc6_target+100 and zc7_now<zc7_target+100: #20 ADC tolerance
            #Do: stop lifting process
                lift_ena = 0
                lift_flag = 0
                # print("masuk")
                # control_mode = 0
                # cs_lamp = 0
            elif zc6_now > zc6_target and zc7_now > zc6_target: 
                #Do: lift down!
                # print("nyangkut")
                lift_ena = 2
                # cs_lamp = 1

        else: #stop lifting
            lift_ena = 0
            if zc6_target_bef != 4690 and zc6_target_bef != 4630: 
                lift_flag = 1

    elif lifting_mode == 1: #mid lift
        zc6_target = 4690 + 124
        zc7_target = 4617 + 157
        if lift_flag: #go lifting
            if zc6_now>5200 or zc7_now>5200: 
                zc6_target = 5150 + 124
                zc7_target = 5075  + 157
            zc6_target_bef = zc6_target

            if zc6_now>zc6_target-20 and zc7_now>zc7_target-20 and zc6_now<zc6_target+20 and zc7_now<zc7_target+20: #20 ADC tolerance
            #Do: stop lifting process
                lift_ena = 0
                lift_flag = 0
            elif zc6_now < zc6_target and zc7_now < zc6_target: 
                #Do: lift up!
                lift_ena = 1
            elif zc6_now > zc6_target and zc7_now > zc6_target: 
                #Do: lift down!
                lift_ena = 2

        else: #stop lifting
            lift_ena = 0
            if zc6_target_bef != 4690 + 124 and zc6_target_bef != 4617 + 157: 
                lift_flag = 1
    
    elif lifting_mode == 2: #up lift
        zc6_target = 5300
        zc7_target = 5300
        if lift_flag: #go lifting
            zc6_target_bef = zc6_target

            if zc6_now>zc6_target-10 and zc7_now>zc7_target-10: #20 ADC tolerance
            #Do: stop lifting process
                lift_ena = 0
                lift_flag = 0
                # print("masuk")


            elif zc6_now < zc6_target and zc7_now < zc6_target: 
                #Do: lift up!
                lift_ena = 1
                # print("masuk")


        else: #stop lifting
            lift_ena = 0
            if zc6_target_bef != 5300: 
                lift_flag = 1


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
                 dock_X, dock_Y, dock_yaw, undock_X, undock_Y, undock_yaw)                 
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
        finish_flag = 0
        # if state['v']>0.2: 
        #     lifting_mode = 1 #in normal cond, take mid lift
        #     lifting_command(lifting_mode)
    
    elif control_mode==1: 
        #DO: give movement check action to the steering angle (move right-left-straight)
        # print("Mode 1 Received! Do steering check")
        if steer_angle_bef < 0.052 and steer_angle_bef > -0.052: #between 3 and -3 degrees
        # if count < 100:
            # in this condition, move the steering angle to the right 20 degrees
            lever_mode = 2 #set forward movement
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
            if not finish_flag:
                finish_flag, cs_long, cs_lat, cs_brake = controller.calculate_path_following_signal(xpath,
                                                                ypath, state['v'],
                                                                state['yaw'])
            cs_lamp = 1
            if finish_flag:
                cs_brake = 1
                cs_long = 0
                # cs_lat = 0
                count +=1
                if count>200:
                    lever_mode = 0 #set neutral and stop
                    cs_lamp = 0  
                    cs_lat = 0
                    cs_long = 0
                    cs_brake = 0
                    count = 0

            # if count > 200: 
                # print("Finished! Get back to stand by mode")
                # control_mode = 0 
                # lever_mode = 0 #set neutral and stop
                # cs_lamp = 0  
                # cs_lat = 0
                # cs_long = 0
                # cs_brake = 0
        else:
            print("[INFO] [", rospy.Time.now(), "] State Variable has not received yet. Cannot start autonomous mode.")
        
        # if state['v']>0.2: 
        lifting_mode = 1 #in normal cond, take mid lift
        lifting_command(lifting_mode)

        # count +=1
    
    elif control_mode == 4: 
        #DO: backward motion with point stabilization control algorithm
        # important! make sure the movement mode is set to backward
        # will work only if the state variable (/Data_Conv) topic is available
        if (RECEIVED_STATE_VAL):
            # debug message
            # print("Autonomous Point Stabilization Mode!")
            lever_mode = 2 #set reverse movement
            ### Calculate the control signal

            #!! Use this for only 1 stage
            # if not finish_flag: 
            #     finish_flag, cs_long, cs_lat, cs_brake, filter_flag = controller.calculate_point_stab_signal(state['x'],
            #                                                     state['y'], state['v'],
            #                                                     state['yaw'], zc6_now)

            #!! Use this one for 2 stage PS 
            target_pos = [x1, y1, yaw1, x2, y2, yaw2]
            if not finish_flag: 
                finish_flag, cs_long, cs_lat, cs_brake, stage_flag = controller.calculate_point_stab_signal_2ndstage(state['x'],
                                                                state['y'], state['v'],
                                                                state['yaw'], zc6_now,target_pos, stage)

            if stage_flag == 1: #go to the next point
                stage = 1

            # if controller.errx_ < 1:
            #     cs_lat = beta * cs_lat_last + (1-beta)*cs_lat
            # else:
            #     cs_lat_last = cs_lat
            cs_lamp = 1
            if finish_flag:
                stage_flag = 0
                stage = 0
                cs_brake = 2
                # cs_long = 0
                lever_mode = 0 #set neutral and stop
                cs_lat = 0
                count +=1
                if count>30:
                    cs_lat = 0
                    cs_brake = 0
                    cs_lamp = 0
                    count = 0
                # elif count>35:
                #     lever_mode = 0 #set neutral and stop
                #     cs_lamp = 0  
                #     cs_lat = 0
                #     # cs_long = 0
                #     cs_brake = 0
                #     count = 0
            # if finish_flag == 1: 
            #     # print("Finished! Get back to stand by mode")
            #     lever_mode = 0 #set neutral and stop
            #     control_mode = 0 
            #     cs_lamp = 0  
            #     cs_lat = 0
            #     cs_long = 0
            #     cs_brake = 0
        else:
            print("[INFO] [", rospy.Time.now(), "] State Variable has not received yet. Cannot start autonomous mode.")
        
        # check container to decide lift position
        # if state['v']>0.2: 
        #     if zc8_now < 6553: #container exist!
        #         lifting_mode = 2 #lift up!
        #         lifting_command(lifting_mode)
        #     elif zc8_now > 9380: #no container!
        #         lifting_mode = 0 #lift down!
        #         lifting_command(lifting_mode)
        # count +=1
    
    elif control_mode == 5: 
        #DO: backward motion with point stabilization control algorithm
        # important! make sure the movement mode is set to backward
        # will work only if the state variable (/Data_Conv) topic is available
        if (RECEIVED_STATE_VAL):
            # debug message
            # print("Autonomous Docking Mode!")
            lever_mode = 2 #set reverse movement
            ### Calculate the control signal
            if not finish_flag:
                finish_flag, cs_long, cs_lat, cs_brake = controller.calculate_docking_signal(state['x'],
                                                                state['y'], state['v'],
                                                                state['yaw'], zc6_now)
            cs_lamp = 1
            if finish_flag:
                cs_brake = 1
                cs_long = 0
                # cs_lat = 0
                lever_mode = 0 #set neutral and stop
                count +=1
                if count>100:
                    cs_lamp = 0  
                    cs_lat = 0
                    cs_long = 0
                    cs_brake = 0
                    count = 0
            # if finish_flag == 1: 
            #     # print("Finished! Get back to stand by mode")
            #     lever_mode = 0 #set neutral and stop
            #     control_mode = 0 
            #     cs_lamp = 0  
            #     cs_lat = 0
            #     cs_long = 0
            #     cs_brake = 0
        else:
            print("[INFO] [", rospy.Time.now(), "] State Variable has not received yet. Cannot start autonomous mode.")
        
        # count += 1
    
    elif control_mode == 6: 
        #DO: lifting up!
        # will work only if the state variable (/Data_Conv) topic is available
        zc6_target = 5100
        zc7_target = 5114
        if zc6_now>zc6_target-10 and zc7_now>zc7_target-10: #20 ADC tolerance
        #Do: stop lifting process
            lift_ena = 0
            control_mode = 0
            cs_lamp = 0
        elif zc6_now < zc6_target and zc7_now < zc6_target: 
            #Do: lift up!
            lift_ena = 1
            cs_lamp = 1
        # zc6_target = 5003#5120
        # zc7_target = 5114#5180
        # print("here")

        # lifting_mode = 2
        # lifting_command(lifting_mode)
        # publish the message
        cs_lat = 0
        cs_long = 2
        cs_brake = 0
        lever_mode = 0

    elif control_mode == 7: 
        #DO: lifting mid!
        # will work only if the state variable (/Data_Conv) topic is available
        # if zc6_now>5200 or zc7_now>5200: 
        #     zc6_target = 4990 + 124
        #     zc7_target = 5035  + 157
        # # elif zc6_now<5200 and zc7_now<5200 and cs_lamp == 0:
        # #     zc6_target = 4540 + 124
        # #     zc7_target = 4578 + 157

        # if zc6_now>zc6_target-20 and zc7_now>zc7_target-20 and zc6_now<zc6_target+20 and zc7_now<zc7_target+20: #20 ADC tolerance
        # #Do: stop lifting process
        #     lift_ena = 0
        #     control_mode = 0
        #     cs_lamp = 0
        # elif zc6_now < zc6_target and zc7_now < zc6_target: 
        #     #Do: lift up!
        #     lift_ena = 1
        #     cs_lamp = 1
        # elif zc6_now > zc6_target and zc7_now > zc6_target: 
        #     #Do: lift down!
        #     lift_ena = 2
        #     cs_lamp = 1
        lifting_mode = 1
        lifting_command(lifting_mode)
        # publish the message
        cs_lat = 0
        cs_long = 0
        cs_brake = 0
        lever_mode = 0
    
    elif control_mode == 8: 
        #DO: lifting down!
        # will work only if the state variable (/Data_Conv) topic is available
        if zc6_now>5000 or zc7_now>5000:
            # zc6_target = 4990
            # zc7_target = 5035
            zc6_target = 4624#4540
            zc7_target = 4666#4578
        # elif zc6_now<5200 and zc7_now<5200 and cs_lamp == 0:
        #     zc6_target = 4540
        #     zc7_target = 4578

        if zc6_now<zc6_target+70 and zc7_now<zc7_target+40: #20 ADC tolerance
        #Do: stop lifting process
            lift_ena = 0
            control_mode = 0
            cs_lamp = 0
        elif zc6_now > zc6_target and zc7_now > zc6_target: 
            #Do: lift down!
            lift_ena = 2
            cs_lamp = 1
        # lifting_mode = 0 #lift down!
        # lifting_command(lifting_mode)
        # publish the message
        cs_lat = 0
        cs_long = 0
        cs_brake = 0
        lever_mode = 0
    
    elif control_mode == 9: 
        #DO: undocking!
        # will work only if the state variable (/Data_Conv) topic is available
        if (RECEIVED_STATE_VAL):
            # debug message
            # print("Autonomous unDocking Mode!")
            lever_mode = 1 #set forward movement
            ### Calculate the control signal
            if not finish_flag:
                finish_flag, cs_long, cs_lat, cs_brake = controller.calculate_undocking_signal(state['x'],
                                                                state['y'], state['v'],
                                                                state['yaw'])
            cs_lamp = 1
            if finish_flag:
                cs_brake = 2
                cs_long = 0
                cs_lat = 0
                count +=1
                if count>100:
                    lever_mode = 0 #set neutral and stop
                    cs_lamp = 0  
                    cs_lat = 0
                    cs_long = 0
                    cs_brake = 0
                    count = 0
            # if finish_flag == 1: 
            # # if count > 200:
            #     # print("Finished! Get back to stand by mode")
            #     lever_mode = 0 #set neutral and stop
            #     control_mode = 0 
            #     cs_lamp = 0  
            #     cs_lat = 0
            #     cs_long = 0
            #     cs_brake = 0
        else:
            print("[INFO] [", rospy.Time.now(), "] State Variable has not received yet. Cannot start autonomous mode.")
        
        # count += 1
    
    elif control_mode == 10: 
        #DO: take/place container!
        if zc8_now < 6553: #container exist!
            lifting_mode = 0 #place the container
            lifting_command(lifting_mode)
            cs_lamp = 1
            if not lift_flag: 
                cs_lamp = 0
                control_mode = 0
            lever_mode = 0 #set neutral and stop
            cs_lat = 0
            cs_long = 0
            cs_brake = 0
       
        elif zc8_now > 9380: #no container!
            lifting_mode = 2 #take the container
            lifting_command(lifting_mode)
            cs_lamp = 1
            if not lift_flag: 
                cs_lamp = 0
                control_mode = 0
            lever_mode = 0 #set neutral and stop
            cs_lat = 0
            cs_long = 0
            cs_brake = 0

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
    pub_msg.pose.covariance[23] = filter_flag
    pub_msg.pose.covariance[24] = stage
    # print("errx, erry, e_yaw (rad), e_yaw (deg), cs_lat, cs_long")
    # print(round(controller.errx_,5), round(np.degrees(controller.erryaw_),5),\
    #     round(cs_lat, controller.erry_,5))
    # print("node check")
    print(stage, round(controller.errx_,2), round(np.degrees(controller.erryaw_),1), round(np.degrees(cs_lat),1), round(controller.erry_,3), cs_long)
    # Publish the message
    pub.publish(pub_msg)

    ### Wait until the next loop
    rate.sleep()