# CTT Path Tracking
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
pub_msg.header.frame_id = 'CTT_Path_Followint_control'
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
X_pos_t = 17
Y_pos_t = 21
yaw_t = 0.5*3.14
yaw_h = 0.5*3.14
# vehicle parameters
l_t = 13.75
w_t = 2.682
l_h = 3.533
w_h = 2.682
# sigmoid curve parameters
X = [X_pos_t]
Y = [Y_pos_t]
Dx = 49
Dy = 5.5
k1 = 0.2
k2 = 24.5
inc = 100
rot_x = []
rot_y = []
vmax = 2.5 #in m/s
# Target Posision
X_target = 11.5
Y_target = 70

# live plot, take a cup of coffee
print("Waiting.....")
time.sleep(5) #wait for live plot node (start it manually)
print("Simulation Start!")

# create sigmoid waypoints
rot_x, rot_y = CTT.create_sigmoid_wp(X,Y,inc,Dx,Dy,k1,k2)

# calculate yaw, speed, time, curvature
curvature, yaw_wp, arc_length, V_curv, dtr = CTT.calc_time_trajectory(rot_x, rot_y, vmax)

# Stanley parameter
# Kondisi awal untuk penjejak lintasan
xstart = [rot_x[0]]
ystart = [rot_y[0]]
v_ref = []
omega = []
yaw = [0.5*3.14]
t = [0.001]
# dt_sim = 0.1 #15/20 
dt = []
b = l_t/2
delta = []
i = 0
# time_base = time.time()

# calculating time reference for trajectory tracking
dt_tr = np.zeros(len(dtr))

for i in range(len(dtr)): 
  dt_tr[i] = sum(np.array(dtr[0:i]))

# sub = rospy.Subscriber('/vehicle_state', State_Estimator, main_function)
# rospy.spin()

while True:
    # t_now = time.time() - time_base
    # dt_sim = 0.01
    V_AV, cs_steer = CTT.calculate_stanley_control_path(X_pos_t, Y_pos_t, yaw,  rot_x, rot_y, V_curv, yaw_wp)
    X_pos_t, Y_pos_t, yaw = CTT.update_AV_position(X_pos_t, Y_pos_t, V_AV, yaw, cs_steer, dt_sim, l_t)

    print("Run Ruuuuuuuuuuuuuuun!")

    # Store calculated value to pub_msg
    # AV Actual   
    pub_msg.actual_x_av = X_pos_t
    pub_msg.actual_y_av = Y_pos_t
    pub_msg.actual_yaw_av = yaw
    pub_msg.actual_speed_av = V_AV
    pub_msg.actual_steer_av = cs_steer
    # pub_msg.actual_action_av = Action_Game
    pub_msg.path_x_av = rot_x
    pub_msg.path_y_av = rot_y
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

    rate.sleep()
    if Y_pos_t > rot_y[-1]:
      break
    # time.sleep(1)

print("simulation done")