# CTT Point Stabilization
# Husnul Amri (2021)
# github : @moezeus
# adapted from Thesis Works

import numpy as np
import time
import CTT_Sim_Func as CTT
# from thesis.msg import Autonomous_Game
import rospy
from thesis.msg import Game_Theory_Logger
from thesis.msg import State_Estimator
# import rospy
# import os

# ROS thing
rospy.init_node('controller')
freq = 20 # Hz
# pub = rospy.Publisher('/game_theory_AV', Autonomous_Game, queue_size=1)
pub = rospy.Publisher('/CTT_movement', Game_Theory_Logger, queue_size=1)
rate = rospy.Rate(freq) # Hz
# pub_msg = Autonomous_Game()
pub_msg = Game_Theory_Logger()
pub_msg.header.frame_id = 'CTT_Point_Stabilization_control'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()

dt_sim = 1/freq

## parameter initialization
# UTM coordinates
X_utm = [558103.34,558082.38, 558082.34, 558082.34, 558103.22, 558103.30, 558103.34]
Y_utm = [9158033.03, 9158033.05, 9157996.57, 9157931.34, 9157931.32, 9157996.55, 9158033.03]
# Resized coordinates (move the baseline to zero)
X_res = np.array(X_utm[0:]) - min(X_utm)
Y_res = np.array(Y_utm[0:]) - min(Y_utm)
# vehicle initial position
X_pos_t = 6#11.5
Y_pos_t = 21#10.875
yaw_t = 0.5*3.14
yaw_h = 0.5*3.14
# vehicle parameters
l_t = 13.75
w_t = 2.682
l_h = 3.533
w_h = 2.682
# vehicle target position
X_target = 6
Y_target = 9.875
yaw_target = yaw_t
# Point Stabilization gain (Lyapunov)
k_rho = 0.1
k_alpha = 0.2 
# initial error position and orientation
rho_ = np.sqrt((X_pos_t-X_target)**2+(Y_pos_t-Y_target)**2)
# vi_ = np.arctan2(-(Y_pos_t-Y_target),-(X_pos_t-X_target))-yaw_target+10**(-32)
alpha_ = np.arctan2(-(Y_pos_t-Y_target),-(X_pos_t-X_target))-yaw_t+10**(-32)

# live plot, take a cup of coffee
print("Waiting.....")
time.sleep(5) #wait for live plot node (start it manually)
print("Simulation Start!")

while True:
    # t_now = time.time() - time_base
    # dt_sim = 0.01
    V_AV, cs_steer, rho_, alpha_ = CTT.calculate_point_stabilization(X_pos_t, Y_pos_t, yaw_t, X_target, Y_target, yaw_target, k_rho, k_alpha, l_t)
    X_pos_t, Y_pos_t, yaw_t = CTT.update_AV_position(X_pos_t, Y_pos_t, V_AV, yaw_t, cs_steer, dt_sim, l_t)

    print(rho_)

    # Store calculated value to pub_msg
    # AV Actual   
    pub_msg.actual_x_av = X_pos_t
    pub_msg.actual_y_av = Y_pos_t
    pub_msg.actual_yaw_av = yaw_t
    pub_msg.actual_speed_av = V_AV
    pub_msg.actual_steer_av = cs_steer
    # pub_msg.actual_action_av = Action_Game
    # pub_msg.path_x_av = X_pos_t
    # pub_msg.path_y_av = Y_pos_t
    # # AV Target
    pub_msg.target_x_av = X_target
    pub_msg.target_y_av = Y_target
    # pub_msg.target_yaw_av = yaw_AV[iterate_waypoints]
    # pub_msg.target_speed_av = V_ref_tr[iterate_waypoints]
    # pub_msg.target_steer_av = cs_steer
    # # Other Vehicle
    # pub_msg.actual_x_other = X_oth
    # pub_msg.actual_y_other = Y_oth
    # pub_msg.actual_yaw_other = yaw_oth
    # pub_msg.actual_speed_other = V_oth
    # # Game Theory 
    # pub_msg.game_reward = Reward
    # pub_msg.game_obstacle_x = params.x_br
    # pub_msg.game_obstacle_y = params.y_br
    # pub_msg.game_case_other = others_case
    # pub_msg.game_final_x = params.final_x
    # pub_msg.game_final_y = params.final_y
    # # Header
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.header.seq += 1
    pub.publish(pub_msg)

    # end simulation case
    if rho_ <= 0.1: 
        break

    rate.sleep()
    # time.sleep(1)

print("Simulation Done!")