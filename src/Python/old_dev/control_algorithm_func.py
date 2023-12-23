# This code is written in Python 3 environment
#!/usr/bin/env python3
# CINOVASI - HAM MAF
# This node is intended to calculate control signal, mainly steering and speed set point 
# Forward movement is using Stanley Control and Sigmoid waypoints, while backward movement is using
#       Point Stabilization control

import numpy as np

class Controller(object):
    def __init__(self, k_rho, k_alpha, kv_alpha, ks, kv, sat_lat, sat_long,\
                 waypoints, stop_limit, PSback_X, PSback_Y, PSback_yaw, length,\
                 dock_X, dock_Y, dock_yaw):
        # In this version, the integral term will be clamped based on the
        # saturation value and the feed-forward term

        self.name = "Control Function AGV CTT"

        # The parameters of the longitudinal controller
        self._k_rho = k_rho
        self._k_alpha = k_alpha
        self._kv_alpha = kv_alpha
        self._ks = ks
        self._kv = kv
        self._sat_long_max = sat_long
        self._sat_lat_max = np.fmax(sat_lat[0], sat_lat[1])
        self._sat_lat_min = np.fmin(sat_lat[0], sat_lat[1])
        self.cs_steer_now = 0
        self.cs_steer_bef = 0
        
        # Waypoints (n, 5) -> x, y, yaw, v, curvature
        self._waypoints = waypoints
        self._closest_idx = 0
        self.stop_limit = stop_limit
        self.finish_flag = 0

        # point stab parameter (backward)
        self.backward_point_stab_x = PSback_X
        self.backward_point_stab_y = PSback_Y
        self.yaw_point_stab = PSback_yaw
        self._length = length

        # docking parameter
        self.dock_point_stab_x = dock_X
        self.dock_point_stab_y = dock_Y
        self.yaw_dock = dock_yaw

        # stanley init
        self._e_yaw = 0
        self._e_lat = 0

        # point stab init
        self.rho_ = 0
        self.errx_ = 0
        self.erry_ = 0
        self.vi_ = 0
        self.alpha_ = 0
        self.omega = 0
        self.erryaw_ = 0
        self.k_omega = 0.3
        self.ps_cond = 0 #check if ps has fulfilled some condition
    
    def _feed_forward_lateral(self, idx):
        temp = self._length * self._waypoints[idx, 4]
        if np.abs(temp) >= 1. : 
            temp = np.sign(temp)
        return np.fmax(np.fmin(np.arcsin(temp), self._sat_lat_max),self._sat_lat_min)

    def calculate_path_following_signal(self, X, Y, v, yaw): 
        # Find the closest waypoint
        self.finish_flag = 0
        X_wp = self._waypoints[:,0]
        Y_wp = self._waypoints[:,1]
        yaw_wp = self._waypoints[:,2]
        v_wp = self._waypoints[:,3]
        # self._closest_idx = np.argmin(np.sum(np.square(self._waypoints[:, :2] - np.array([x, y])), axis=-1))
        T_tr_idx = np.argmin((np.array(X_wp)[0:]-np.array(X))**2 + (np.array(Y_wp)[0:]-np.array(Y))**2)
        # T_tr_idx = 0
        # always track the first waypoints
        if T_tr_idx == 0: 
            T_tr_idx = 1
        print(T_tr_idx)
        V_AV = v_wp[T_tr_idx]
        cs_long = min(V_AV, self._sat_long_max)
        # delta (Use Stanley!)
        self._e_yaw = yaw_wp[T_tr_idx] - yaw
        self._e_yaw = (self._e_yaw + np.pi) % (2 * np.pi) - np.pi # Wrap the angle to [-pi, pi)
        _dtrx = X_wp[T_tr_idx] - X_wp[T_tr_idx-1]
        _dtry = Y_wp[T_tr_idx] - Y_wp[T_tr_idx-1]
        c = _dtrx * Y_wp[T_tr_idx-1] -  _dtry * X_wp[T_tr_idx-1]
        self._e_lat = ((X*_dtry)+ c - (Y*_dtrx))/np.sqrt(_dtrx**2+_dtry**2) + 10**(-32)
        # cs_steer = 5 * self._e_yaw + np.arctan(self._ks*self._e_lat / (self._kv + V_AV))
        cs_steer = 5 * self._e_yaw + np.arctan(0.8*self._e_lat / (self._kv + V_AV))
        # cs_lat = min(max(cs_steer, self._sat_lat_min),self._sat_lat_max) #maximum 45 degree
        ff = self._feed_forward_lateral(T_tr_idx)
        cs_steer = cs_steer + ff
        cs_lat = min(max(cs_steer, self._sat_lat_min),self._sat_lat_max) #maximum 45 degree
        
        # if np.absolute(self._e_lat) < 0.4 and np.absolute(self._e_yaw)<np.radians(2):
        #     cs_lat = 0
        
        # if self.cs_lat < 0.1:

        # if np.degrees(cs_lat)<5 and np.degrees(cs_lat)
        # if np.degrees(cs_lat)<np.degrees(self.cs_steer_bef)+3 and np.degrees(cs_lat)<np.degrees(self.cs_steer_bef)+3:

        # print(self._e_lat)

        if len(self._waypoints) - T_tr_idx < self.stop_limit:
            cs_long = 0
            cs_lat = 0
            self.finish_flag = 1
        
        cs_brake = 0
        print("e_lat, e_yaw, cs_lat, cs_long")
        print(self._e_lat,self._e_yaw, cs_lat, cs_long)
        
        return self.finish_flag, cs_long, cs_lat, cs_brake
    
    def calculate_point_stab_signal(self, x, y, v, yaw): 
        # error calc
        # target yaw is always pi/2 to pi/2, but for good reason in the stabilization
        # we make it -pi/2 to -pi/2 (creating a smooth path), with notes, the speed is set to negative (backward)
        # Thus, this function definition is only valid for backward motion with pi/2 to pi/2 orientation
        # The orientation value is substracted by pi, to change the pi/2 to -pi/2
        self.finish_flag = 0
        self.rho_ = np.sqrt((x-self.backward_point_stab_x)**2+(y-self.backward_point_stab_y)**2)
        self.errx_ = np.sqrt((x-self.backward_point_stab_x)**2)
        self.erry_ = np.sqrt((y-self.backward_point_stab_y)**2)
        self.erryaw_ = np.absolute(yaw - self.yaw_point_stab)
        self.vi_ = np.arctan2(-(y-self.backward_point_stab_y),-(x-self.backward_point_stab_x))-(self.yaw_point_stab-np.pi)+10**(-32)
        self.alpha_ = np.arctan2(-(y-self.backward_point_stab_y),-(x-self.backward_point_stab_x))-(yaw-np.pi)#+10**(-32)
        # linear v
        v_ps = (self._k_rho * self.rho_ * np.cos(self.alpha_))
        if np.absolute(self.alpha_ )< 0.01:
            self.omega = (self._k_alpha * self.alpha_ + self._kv_alpha * self._k_rho * (self.vi_ + self.alpha_) \
                        * np.cos(self.alpha_) * 1)
        else:
            self.omega = (self._k_alpha * self.alpha_ + self._kv_alpha * self._k_rho * (self.vi_ + self.alpha_) \
                        * np.cos(self.alpha_) * (np.sin(self.alpha_)/self.alpha_))
        # self.omega = (self._k_alpha * self.alpha_ + self._kv_alpha * self._k_rho * (self.vi_ + self.alpha_) \
        #     * np.cos(self.alpha_) * (np.sin(self.alpha_)/self.alpha_))
        self.omega = self.k_omega * self.omega
        cs_lat = -np.arctan((self._length/v_ps)*(self.omega))
        # cs_lat = -np.arctan((self._length/1)*(self.omega))
        cs_lat = min(max(cs_lat, self._sat_lat_min),self._sat_lat_max)

        cs_long = min(v_ps, self._sat_long_max)
        
        # if self.errx_ < 0.15 and np.absolute(self.vi_)<np.radians(1):
        # if self.rho_ < 5:
        #     print("rho in")
        # if self.errx_ < 0.50:
        #     print("x in")
        # if np.absolute(self.erryaw_)<np.radians(10):
        #     print("yaw in")

        # if self.ps_cond==0 and self.errx_ < 0.50 and np.absolute(self.erryaw_)<np.radians(20):
        #     self.ps_cond==1 #first cond fulfilled!
        #     print("con1")
        
        # elif self.ps_cond==1: #keep straight steer!
        #     if self.rho_ < 1:
        #         self.ps_cond = 2 #finish!
        #         print("con2")
        #     cs_lat = 0
        #     print("con1")
        #     # self.finish_flag = 1
        
        # elif self.ps_cond==2:
        #     cs_long = 0
        #     cs_lat = 0
        #     self.finish_flag = 1
        #     self.ps_cond=0
        #     print("con3")

        if self.rho_ < 10 and self.errx_ < 0.05 and np.absolute(self.erryaw_)<np.radians(0.3):
            cs_long = 0
            cs_lat = 0
            self.finish_flag = 1
        
        cs_brake = 0
        # print("e_x, e_y, e_yaw, cs_lat, cs_long")
        # print(self.errx_, self.erry_, self.vi_, cs_lat, cs_long)

        return self.finish_flag, cs_long, cs_lat, cs_brake
    
    def calculate_docking_signal(self, x, y, v, yaw): 
        # error calc
        # target yaw is always pi/2 to pi/2, but for good reason in the stabilization
        # we make it -pi/2 to -pi/2 (creating a smooth path), with notes, the speed is set to negative (backward)
        # Thus, this function definition is only valid for backward motion with pi/2 to pi/2 orientation
        # The orientation value is substracted by pi, to change the pi/2 to -pi/2
        self.finish_flag = 0
        self.rho_ = np.sqrt((x-self.dock_point_stab_x)**2+(y-self.dock_point_stab_y)**2)
        self.errx_ = np.sqrt((x-self.dock_point_stab_x)**2)
        self.erryaw_ = np.absolute(yaw - self.yaw_dock)
        self.vi_ = np.arctan2(-(y-self.dock_point_stab_y),-(x-self.dock_point_stab_x))-(self.yaw_dock-np.pi)+10**(-32)
        self.alpha_ = np.arctan2(-(y-self.dock_point_stab_y),-(x-self.dock_point_stab_x))-(yaw-np.pi)+10**(-32)
        # linear v
        v_ps = (self._k_rho * self.rho_ * np.cos(self.alpha_))
        self.omega = (self._k_alpha * self.alpha_ + self._kv_alpha * self._k_rho * (self.vi_ + self.alpha_)\
             * np.cos(self.alpha_) * (np.sin(self.alpha_)/self.alpha_))
        cs_lat = -np.arctan((self._length/v_ps)*(self.omega))
        cs_lat = min(max(cs_lat, self._sat_lat_min),self._sat_lat_max)

        cs_long = min(v_ps, self._sat_long_max)
        cs_lat = np.radians(0)

        if self.erryaw_ > np.radians(0.3):
            err_orient = yaw - self.yaw_dock
            if err_orient < 0:
                cs_lat = np.radians(-5)
            elif err_orient > 0:
                cs_lat = np.radians(5)

        # if self.rho_ < 0.15 and np.absolute(self.vi_)<np.radians(2):
        if self.rho_ < 0.05 : # and np.absolute(self.erryaw_)<np.radians():
            cs_long = 0
            cs_lat = 0
            self.finish_flag = 1
        
        cs_brake = 0
        # print("e_x, e_y, e_yaw, cs_lat, cs_long")
        # print(self.errx_, self.erry_, self.vi_, cs_lat, cs_long)
        
        return self.finish_flag, cs_long, cs_lat, cs_brake
#########################################################################################
# reserved - old code
# path following, somehow this fancy one doesnt work
# if self._closest_idx == 0:
        #     self._closest_idx += 1
        
        # cs_long = self._waypoints[self._closest_idx,3] #speed from the waypoint
        # cs_long = min(cs_long, self._sat_long_max) #choose the lowest value based on maximum speed

        # _e_yaw = self._waypoints[self._closest_idx, 2] - yaw
        # _e_yaw = (_e_yaw + np.pi) % (2 * np.pi) - np.pi # Wrap the angle to [-pi, pi)
        # _dtrx = self._waypoints[self._closest_idx, 0] - self._waypoints[self._closest_idx-1, 0]
        # _dtry = self._waypoints[self._closest_idx, 1] - self._waypoints[self._closest_idx-1, 1]
        # c = _dtrx * self._waypoints[self._closest_idx-1, 1] -  _dtry * self._waypoints[self._closest_idx-1, 0]
        # _e_lat = ((x*_dtry)+ c - (y*_dtrx))/np.sqrt(_dtrx**2+_dtry**2) + 10**(-32)
        # cs_lat = _e_yaw + np.arctan(self._ks*_e_lat / (self._kv + self._waypoints[self._closest_idx,3]))
        # # cs_lat = _e_yaw + np.arctan(1*_e_lat / (1 + self._waypoints[self._closest_idx,3]))
        # cs_lat = min(max(cs_lat, self._sat_lat_max),self._sat_lat_min) 
        # print("here")
        # print(self._closest_idx, self._waypoints[self._closest_idx, 0], self._waypoints[self._closest_idx, 1], self._waypoints[self._closest_idx, 2], \
        #             x, y, yaw)

        # if len(self._waypoints) - self._closest_idx < self.stop_limit:
        #     cs_long = 0
        #     cs_lat = 0
        #     self.finish_flag = 1