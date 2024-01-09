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
                 dock_X, dock_Y, dock_yaw, undock_X, undock_Y, undock_yaw):
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
        self.stage_flag = 0


        # point stab parameter (backward)
        self.backward_point_stab_x = PSback_X
        self.backward_point_stab_y = PSback_Y
        self.yaw_point_stab = PSback_yaw
        self._length = length

        # docking parameter
        self.dock_point_stab_x = dock_X
        self.dock_point_stab_y = dock_Y
        self.yaw_dock = dock_yaw

        # undocking parameter
        self.undock_point_stab_x = undock_X
        self.undock_point_stab_y = undock_Y
        self.yaw_undock = undock_yaw

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
        self.count = 0
    
    def wrap_angle(self, angle): 
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _feed_forward_lateral(self, idx):
        temp = self._length * self._waypoints[idx, 4]
        if np.abs(temp) >= 1. : 
            temp = np.sign(temp)
        return np.fmax(np.fmin(np.arcsin(temp), self._sat_lat_max),self._sat_lat_min)

    def calculate_path_following_signal(self, X, Y, v, yaw): 
        # Find the closest waypoint
        self.finish_flag = 0
        X_wp = self._waypoints[:,0] 
        Y_wp = self._waypoints[:,1] + 11.92
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
        # cs_long = min(V_AV, 1.8)
        # delta (Use Stanley!)
        self._e_yaw = yaw_wp[T_tr_idx] - yaw
        self._e_yaw = (self._e_yaw + np.pi) % (2 * np.pi) - np.pi # Wrap the angle to [-pi, pi)
        _dtrx = X_wp[T_tr_idx] - X_wp[T_tr_idx-1]
        _dtry = Y_wp[T_tr_idx] - Y_wp[T_tr_idx-1]
        c = _dtrx * Y_wp[T_tr_idx-1] -  _dtry * X_wp[T_tr_idx-1]
        self._e_lat = ((X*_dtry)+ c - (Y*_dtrx))/np.sqrt(_dtrx**2+_dtry**2) + 10**(-32)
        # cs_steer = 5 * self._e_yaw + np.arctan(self._ks*self._e_lat / (self._kv + V_AV))
        cs_steer = 2 * self._e_yaw + np.arctan(1*self._e_lat / (self._kv + V_AV))
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
    
    def calculate_point_stab_signal_2ndstage(self, x, y, v, yaw, zc6, target, stage): 
        # target = [x1, y1, yaw1, x2, y2, yaw2]
        # x1, y1, yaw1 = target[0], target[1], target[2]
        # x2, y2, yaw2 = target[3], target[4], target[5]

        if stage: 
            point_stab_x, point_stab_y, point_stab_yaw  = target[3], target[4], target[5]
        else:
            point_stab_x, point_stab_y, point_stab_yaw  = target[0], target[1], target[2]

        self.finish_flag = 0
        self.rho_ = np.sqrt((x-point_stab_x)**2+(y-point_stab_y)**2)
        if zc6 > 5000:
            back_ps_x = point_stab_x + 0
        else:
            back_ps_x = point_stab_x - 0

        self.errx_ = np.sqrt((x-point_stab_x)**2)
        self.erry_ = np.sqrt((y-point_stab_y)**2)
        self.erryaw_ = np.absolute(yaw - point_stab_yaw)
        self.vi_ = np.arctan2(-(y-point_stab_y),-(x-point_stab_x))-(point_stab_yaw-np.pi)+10**(-32)
        self.alpha_ = np.arctan2(-(y-point_stab_y),-(x-point_stab_x))-(yaw-np.pi)+10**(-32) 

        v_ps = (self._k_rho * self.rho_ * np.cos(self.alpha_))
        if np.absolute(self.alpha_ )< 0.01:
            self.omega = (self._k_alpha * self.alpha_ + self._kv_alpha * self._k_rho * (self.vi_ + self.alpha_) \
                        * np.cos(self.alpha_) * 1)
        else:
            self.omega = (self._k_alpha * self.alpha_ + self._kv_alpha * self._k_rho * (self.vi_ + self.alpha_) \
                        * np.cos(self.alpha_) * (np.sin(self.alpha_)/self.alpha_))

        self.omega = self.k_omega * self.omega
        
        v_steer = v_ps
        self.omega = self.k_omega * self.omega
        cs_lat = -np.arctan((self._length/v_steer)*(self.omega))
        
        cs_lat = min(max(cs_lat, (np.radians(-45))),np.radians(45))
        cs_long = min(v_ps, 3)

        if stage == 0:
            if self.errx_ < 0.15:
                self.stage_flag = 1
        elif stage ==1: 
            if abs(self.errx_) < 0.05 and self.rho_<7:
                err_orient = yaw - point_stab_yaw
                # if err_orient < 0: 
                #     # cs_lat = np.radians(-5)
                #     cs_lat = err_orient * -7
                # elif err_orient > 0:
                #     # cs_lat = np.radians(5)
                #     cs_lat = err_orient * 7
            else:
                cs_lat = cs_lat

            if self.errx_ < 0.02 and np.absolute(self.erryaw_)<np.radians(1) and self.rho_<3:
                cs_long = 0
                cs_lat = 0
                filter_flag = 0
                self.finish_flag = 1
             
        cs_brake = 0

        return self.finish_flag, cs_long, cs_lat, cs_brake, self.stage_flag
    
    def calculate_point_stab_signal(self, x, y, v, yaw, zc6): 
        # error calc
        # target yaw is always pi/2 to pi/2, but for good reason in the stabilization
        # we make it -pi/2 to -pi/2 (creating a smooth path), with notes, the speed is set to negative (backward)
        # Thus, this function definition is only valid for backward motion with pi/2 to pi/2 orientation
        # The orientation value is substracted by pi, to change the pi/2 to -pi/2
        self.finish_flag = 0
        self.rho_ = np.sqrt((x-self.backward_point_stab_x)**2+(y-self.backward_point_stab_y)**2)
        if zc6 > 5000:
            back_ps_x = self.backward_point_stab_x + 0
        else:
            back_ps_x = self.backward_point_stab_x - 0

        self.errx_ = np.sqrt((x-self.backward_point_stab_x)**2)
        # print(round(x-back_ps_x,3))
        self.erry_ = np.sqrt((y-self.backward_point_stab_y)**2)
        self.erryaw_ = np.absolute(yaw - self.yaw_point_stab)
        self.vi_ = np.arctan2(-(y-self.backward_point_stab_y),-(x-back_ps_x))-(self.yaw_point_stab-np.pi)+10**(-32)
        self.alpha_ = np.arctan2(-(y-self.backward_point_stab_y),-(x-back_ps_x))-(yaw-np.pi)#+10**(-32) 

        # self.errx_ = np.sqrt((x-self.backward_point_stab_x)**2)
        # self.erry_ = np.sqrt((y-self.backward_point_stab_y)**2)
        # self.erryaw_ = np.absolute(yaw - self.yaw_point_stab)
        # self.vi_ = np.arctan2(-(y-self.backward_point_stab_y),-(x-self.backward_point_stab_x))-(self.yaw_point_stab-np.pi)+10**(-32)
        # self.alpha_ = np.arctan2(-(y-self.backward_point_stab_y),-(x-self.backward_point_stab_x))-(yaw-np.pi)#+10**(-32)
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

        # if self.rho_ < 3:
        #     self.k_omega = 0.005
        # else:
        #     self.k_omega = 0.3

        self.omega = self.k_omega * self.omega
        # cs_lat = -np.arctan((self._length/v_ps)*(self.omega))
        
        v_steer = v_ps
        # if v_steer < 0.4:
        #     v_steer = 0.4

        self.omega = self.k_omega * self.omega
        # cs_lat = -np.arctan((self._length/v_steer)*(3/self._length)*(self.omega))
        cs_lat = -np.arctan((self._length/v_steer)*(self.omega))
        
        # cs_lat = min(max(cs_lat, self._sat_lat_min),self._sat_lat_max)
        cs_lat = min(max(cs_lat, (np.radians(-45))),np.radians(45))
        
        # if abs(self.errx_) < 0.1:
        #     err_orient = yaw - self.yaw_point_stab
        #     if err_orient < 0: 
        #         cs_lat = np.radians(-5)
        #     elif err_orient > 0:
        #         cs_lat = np.radians(5)
        # else:
        #     cs_lat = cs_lat

        cs_long = min(v_ps, self._sat_long_max)
        # if self.rho_ < 5:
        #     cs_long = min(v_ps, self._sat_long_max)
        # else: 
        #     cs_long = min(v_ps, 2)

        # if self.rho_ < 5: 
        #     cs_long = min(v_ps, 1)
        
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
        if cs_long < 1: 
            filter_flag = 0
        else:
            filter_flag = 0

        # if self.rho_ < 10 and self.errx_ < 0.03 and np.absolute(self.erryaw_)<np.radians(1):
        # if self.rho_ < 10 and self.errx_ < 0.1 and self.erry_ < 0.02 and np.absolute(self.erryaw_)<np.radians(1):
        if self.rho_ < 5 and self.errx_ < 0.05 and np.absolute(self.erryaw_)<np.radians(0.5):
            cs_long = 0
            cs_lat = 0
            filter_flag = 0
            self.finish_flag = 1
        
        cs_brake = 0
        # print("e_x, e_y, e_yaw, cs_lat, cs_long")
        # print(self.errx_, self.erry_, self.vi_, cs_lat, cs_long)

        return self.finish_flag, cs_long, cs_lat, cs_brake, filter_flag

    def calculate_point_stab_signal_2ndstage_rot(self, x, y, v, yaw, zc6, target, stage): 

        if stage: 
            point_stab_x, point_stab_y, point_stab_yaw  = target[3], target[4], target[5]
        else:
            point_stab_x, point_stab_y, point_stab_yaw  = target[0], target[1], target[2]

        self.finish_flag = 0

        angle_rot = 0.8197

        yawrot = 0.8197 - np.pi/2

        x_target = point_stab_x * np.cos(yawrot) + point_stab_y * np.sin(yawrot)
        y_target = -point_stab_x * np.sin(yawrot) + point_stab_y * np.cos(yawrot)
        yaw_target = point_stab_yaw - yawrot

        # x_target = point_stab_x
        # y_target = point_stab_y
        # yaw_target = point_stab_yaw

        x_rot = x * np.cos(yawrot) + y * np.sin(yawrot)
        y_rot = -x * np.sin(yawrot) + y * np.cos(yawrot)
        yaw_rot = yaw - yawrot

        # x_rot = x
        # y_rot = y
        # yaw_rot = yaw

        self.rho_ = np.sqrt((x_rot-x_target)**2+(y_rot-y_target)**2)
        self.errx_ = abs(x_rot-x_target)
        self.erry_ = np.sqrt((y_rot-y_target)**2)
        self.erryaw_ = yaw_rot - yaw_target
        self.vi_ = np.arctan2(-(y_rot-y_target),-(x_rot-x_target))-(yaw_target-np.pi)+10**(-32)
        self.alpha_ = np.arctan2(-(y_rot-y_target),-(x_rot-x_target))-(yaw_rot-np.pi)#+10**(-32) 

        self.vi_ = self.wrap_angle(self.vi_)
        self.alpha_ = self.wrap_angle(self.alpha_)
        
        v_ps = (self._k_rho * self.rho_ * np.cos(self.alpha_))
    
        self.omega = (self._k_alpha * self.alpha_ + self._kv_alpha * self._k_rho * (self.vi_ + self.alpha_) \
                        * np.cos(self.alpha_) * (np.sin(self.alpha_)/self.alpha_))

        v_steer = v_ps
        # if v_steer < 0.4:
        #     v_steer = 0.4

        self.omega = self.k_omega * self.omega
        # cs_lat = -np.arctan((self._length/v_steer)*(3/self._length)*(self.omega))
        cs_lat = -np.arctan((self._length/v_steer)*(self.omega))

        cs_lat = min(max(cs_lat, (np.radians(-45))),np.radians(45))
        cs_long = min(v_ps, self._sat_long_max)


        if stage == 0:
            if self.errx_ < 0.15:
                self.stage_flag = 1
        elif stage ==1: 
            if abs(self.errx_) < 0.15:
            # if abs(self.errx_) < 0.5:
                err_orient = yaw_target - yaw_rot
                if err_orient < 0: 
                    # cs_lat = np.radians(-5)
                    cs_lat = err_orient * -5
                elif err_orient > 0:
                    # cs_lat = np.radians(5)
                    cs_lat = err_orient * 5
            else:
                cs_lat = cs_lat

            if self.errx_ < 0.1 and np.absolute(self.erryaw_)<np.radians(1):
            # if self.errx_ < 0.5 and np.absolute(self.erryaw_)<np.radians(0.5) and self.erry_ < 0.5:
                cs_long = 0
                cs_lat = 0
                filter_flag = 0
                self.finish_flag = 1
             
        cs_brake = 0

        return self.finish_flag, cs_long, cs_lat, cs_brake, self.stage_flag

    def calculate_point_stab_signal_rot(self, x, y, v, yaw, zc6): 
        # error calc
        # target yaw is always pi/2 to pi/2, but for good reason in the stabilization
        # we make it -pi/2 to -pi/2 (creating a smooth path), with notes, the speed is set to negative (backward)
        # Thus, this function definition is only valid for backward motion with pi/2 to pi/2 orientation
        # The orientation value is substracted by pi, to change the pi/2 to -pi/2
        self.finish_flag = 0

        # yawrot = np.radians(45.8)
        # yawrot = np.radians(45.8) - np.pi/2
        yawrot = 0.8197 - np.pi/2
        yawrot = self.wrap_angle(yawrot)

        x_target = self.backward_point_stab_x * np.cos(yawrot) + self.backward_point_stab_y * np.sin(yawrot)
        y_target = -self.backward_point_stab_x * np.sin(yawrot) + self.backward_point_stab_y * np.cos(yawrot)
        yaw_target = self.yaw_point_stab - yawrot

        x_rot = x * np.cos(yawrot) + y * np.sin(yawrot)
        y_rot = -x * np.sin(yawrot) + y * np.cos(yawrot)
        yaw_rot = yaw - yawrot

        self.rho_ = np.sqrt((x_rot-x_target)**2+(y_rot-y_target)**2)
        self.errx_ = x_rot-x_target
        self.erry_ = np.sqrt((y_rot-y_target)**2)
        self.erryaw_ = yaw_rot - yaw_target
        self.vi_ = np.arctan2(-(y_rot-y_target),-(x_rot-x_target))-(yaw_target-np.pi)+10**(-32)
        self.alpha_ = np.arctan2(-(y_rot-y_target),-(x_rot-x_target))-(yaw_rot-np.pi)#+10**(-32) 

        self.vi_ = self.wrap_angle(self.vi_)
        self.alpha_ = self.wrap_angle(self.alpha_)
        
        v_ps = (self._k_rho * self.rho_ * np.cos(self.alpha_))
        # if np.absolute(self.alpha_ )< 0.01:
        #     self.omega = (self._k_alpha * self.alpha_ + self._kv_alpha * self._k_rho * (self.vi_ + self.alpha_) \
        #                 * np.cos(self.alpha_) * 1)
        # else:
        #     self.omega = (self._k_alpha * self.alpha_ + self._kv_alpha * self._k_rho * (self.vi_ + self.alpha_) \
        #                 * np.cos(self.alpha_) * (np.sin(self.alpha_)/self.alpha_))

        self.omega = (self._k_alpha * self.alpha_ + self._kv_alpha * self._k_rho * (self.vi_ + self.alpha_) \
                        * np.cos(self.alpha_) * (np.sin(self.alpha_)/self.alpha_))

        v_steer = v_ps
        # if v_steer < 0.4:
        #     v_steer = 0.4

        self.omega = self.k_omega * self.omega
        # cs_lat = -np.arctan((self._length/v_steer)*(3/self._length)*(self.omega))
        cs_lat = -np.arctan((self._length/v_steer)*(self.omega))


        # if abs(self.errx_) < 0.05:
        #     err_orient = yaw_rot - yaw_target
        #     cs_lat = 3*err_orient
        # else:
        #     cs_lat = cs_lat

        # cs_lat = min(max(cs_lat, self._sat_lat_min),self._sat_lat_max)
        # cs_lat = min(max(cs_lat, -np.pi/9),np.pi/9)
        cs_lat = min(max(cs_lat, (np.radians(-45))),np.radians(45))
        cs_long = min(v_ps, self._sat_long_max)
        # cs_long = min(v_ps, 3)


        if cs_long < 1: 
            filter_flag = 0
        else:
            filter_flag = 0

        if self.rho_ < 10 and abs(self.errx_) < 0.1 and abs(self.erryaw_)<np.radians(2):
            cs_long = 0
            cs_lat = 0
            filter_flag = 0
            self.finish_flag = 1
        
        cs_brake = 0
        # print("e_x, e_y, e_yaw, cs_lat, cs_long")
        # print(self.errx_, self.erry_, self.vi_, cs_lat, cs_long)

        return self.finish_flag, cs_long, cs_lat, cs_brake, filter_flag

    def calculate_docking_signal(self, x, y, v, yaw, zc6_now): 
        # error calc
        # target yaw is always pi/2 to pi/2, but for good reason in the stabilization
        # we make it -pi/2 to -pi/2 (creating a smooth path), with notes, the speed is set to negative (backward)
        # Thus, this function definition is only valid for backward motion with pi/2 to pi/2 orientation
        # The orientation value is substracted by pi, to change the pi/2 to -pi/2
        self.finish_flag = 0

        self.rho_ = np.sqrt((x-self.dock_point_stab_x)**2+(y-self.dock_point_stab_y)**2)

        if zc6_now>5000: 
            point_stab_y = self.dock_point_stab_y - 0.2
            if self.rho_ < 6: 
                yaw_docking = np.radians(91) ###
            else: 
                yaw_docking = self.yaw_dock
        else:
            point_stab_y = self.dock_point_stab_y
            if self.rho_ < 4: 
                yaw_docking = np.radians(91)
            else: 
                yaw_docking = self.yaw_dock
        
        #self.errx_ = np.sqrt((x-self.dock_point_stab_x)**2)
        # self.errx_ = np.sqrt((x-self.dock_point_stab_x)**2)
        self.erry_ = np.sqrt((y-point_stab_y)**2)
        self.erryaw_ = yaw - yaw_docking#self.yaw_dock
        #self.vi_ = np.arctan2(-(y-point_stab_y),-(x-self.dock_point_stab_x))-(yaw_docking-np.pi)+10**(-32)
        #self.alpha_ = np.arctan2(-(y-point_stab_y),-(x-self.dock_point_stab_x))-(yaw-np.pi)+10**(-32)
        # linear v
        v_ps = (self._k_rho * self.rho_ * np.cos(self.alpha_))
        #self.omega = (self._k_alpha * self.alpha_ + self._kv_alpha * self._k_rho * (self.vi_ + self.alpha_)\
             * np.cos(self.alpha_) * (np.sin(self.alpha_)/self.alpha_))
        #self.omega = self.k_omega * self.omega
        #cs_lat = -np.arctan((self._length/v_ps)*(self.omega))
        # cs_lat = min(max(cs_lat, self._sat_lat_min),self._sat_lat_max)
        #cs_lat = min(max(cs_lat, np.radians(-10)),np.radians(10))

        # cs_long = min(v_ps, self._sat_long_max)
        # v_ps = 1 #slow speed only
        cs_long = min(v_ps, 3)
        
        #Dicoment 20okt 2023
        if self.count>40: 
            v_ps = 1.6 #0.1

        # cs_long = min(v_ps, self._sat_long_max)

        # cs_lat = np.radians(0)

        if abs(self.erryaw_) > np.radians(0.2):
            err_orient = yaw - yaw_docking
            if err_orient < 0:
                cs_lat = np.radians(-4)
            elif err_orient > 0:
                cs_lat = np.radians(4)
        else:
            cs_lat = np.radians(0)



        # if abs(self.erry_) < 11:
        #     err_orient = yaw - self.yaw_dock
        #     if err_orient < 0: 
        #         cs_lat = np.radians(-5)
        #     elif err_orient > 0:
        #         cs_lat = np.radians(5)
        # else:
        #     cs_lat = cs_lat
        # if self.rho_ < 0.15 and np.absolute(self.vi_)<np.radians(2):
        # if self.rho_ < 0.08 : # and np.absolute(self.erryaw_)<np.radians():
        if self.erry_ < 0.05:
            cs_long = 0
            cs_lat = 0
            self.count = 0
            self.finish_flag = 1
        
        cs_brake = 0
        # print("e_x, e_y, e_yaw, cs_lat, cs_long")
        # print(self.errx_, self.erry_, self.vi_, cs_lat, cs_long)
        self.count += 1
        
        return self.finish_flag, cs_long, cs_lat, cs_brake
    
    def calculate_undocking_signal(self, x, y, v, yaw): 
        # error calc
        # target yaw is always pi/2 to pi/2, but for good reason in the stabilization
        # we make it -pi/2 to -pi/2 (creating a smooth path), with notes, the speed is set to negative (backward)
        # Thus, this function definition is only valid for backward motion with pi/2 to pi/2 orientation
        # The orientation value is substracted by pi, to change the pi/2 to -pi/2
        self.finish_flag = 0
        self.rho_ = np.sqrt((x-self.undock_point_stab_x)**2+(y-self.undock_point_stab_y)**2)
        self.errx_ = np.sqrt((x-self.undock_point_stab_x)**2)
        self.erry_ = np.sqrt((y-self.undock_point_stab_y)**2)
        # self.erryaw_ = np.absolute(yaw - self.yaw_undock)
        # linear
        v_ps = 1 #slow speed only
        
        if y > self.dock_point_stab_y + 5: 
            yaw_undocking = np.radians(89.7)
        else: 
            yaw_undocking = self.yaw_undock

        if self.count>35: 
            v_ps = 1.6

        cs_long = min(v_ps, self._sat_long_max)
        cs_lat = np.radians(0)
        self.erryaw_ = np.absolute(yaw - yaw_undocking)

        if self.erryaw_ > np.radians(0.2):
            err_orient = yaw - yaw_undocking
            if err_orient < 0:
                cs_lat = np.radians(4)
            elif err_orient > 0:
                cs_lat = np.radians(-4.5)

        if y < self.dock_point_stab_y + 1:
            cs_lat = np.radians(0)

        # if self.rho_ < 0.15 and np.absolute(self.vi_)<np.radians(2):
        # if self.rho_ < 0.05 : # and np.absolute(self.erryaw_)<np.radians():
        #     cs_long = 0
        #     cs_lat = 0
        #     self.count = 0
        #     self.finish_flag = 1
        if self.erry_ < 0.05:
            cs_long = 0
            cs_lat = 0
            self.count = 0
            self.finish_flag = 1

        cs_brake = 0
        # print("e_x, e_y, e_yaw, cs_lat, cs_long")
        # print(self.errx_, self.erry_, self.vi_, cs_lat, cs_long)
        self.count += 1
        
        return self.finish_flag, cs_long, cs_lat, cs_brake
    
    # def wrap_angle(self, angle): 
    #     return (angle + np.pi) % (2 * np.pi) - np.pi

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