#! /usr/bin/env python3
import matplotlib.pyplot as plt
import rospy
import os, sys
# import tf
# from nav_msgs.msg import Odometry
import numpy as np
from matplotlib.animation import FuncAnimation
from shapely.geometry import Polygon
from geometry_msgs.msg import PoseWithCovarianceStamped

# UTM coordinates
X_utm = [558103.34,558082.38, 558082.34, 558082.34, 558103.22, 558103.30, 558103.34]
Y_utm = [9158033.03, 9158033.05, 9157996.57, 9157931.34, 9157931.32, 9157996.55, 9158033.03]
# initial position
X_pos_t = 558093.84
Y_pos_t = 9158001.32
yaw_t = 0.5*3.14
yaw_h = 0.5*3.14
# target position point stab
PSbackX = rospy.get_param('~PSback_X', 558092.34)
PSbackY = rospy.get_param('~PSback_Y', 9157962.32)
# target position docking
PSdockX = rospy.get_param('~dock_X', 558093.9866405)#558092.34)
PSdockY = rospy.get_param('~dock_Y', 9157950.32)#9157952.32)
# figure
width = 5
height = 10
# docking spec (x,y,orientation,length,width)
# left docking
l_dock_ = [558090.4866405, 9157952.32, 0.5*np.pi, 10, 0.2]
# right docking
r_dock_ = [558096.4866405,  9157952.32, 0.5*np.pi, 10, 0.2]
# lower docking
low_dock_ = [558093.4866405, 9157947.32, np.pi, 6, 0.2]

print(l_dock_[0],l_dock_[1],l_dock_[2],l_dock_[3],l_dock_[4])

class Visualiser:
    def __init__(self):
        self.X_now = 558099.34
        self.Y_now = 9157952.32
        self.yaw_now = yaw_t
        self.yaw_head = yaw_h
        self.control_mode = 0
        # vehicle parameters
        self.l_t = 14.02 #13.75
        self.w_t = 2.682
        self.l_h = 3.533
        self.w_h = 2.682
        waypoints_path = rospy.get_param('~waypoints_path', 'sigmoid160522.npy')
        waypoints_path = os.path.abspath(sys.path[0] + '/../src/waypoints/' + waypoints_path)
        self.waypoints = np.load(waypoints_path)
        self.fig, self.ax = plt.subplots(figsize=(width,height))
        # self.x_wp, self.y_wp = [], []
        # self.fig, self.ax = plt.subplots(figsize=(width,height))
        # self.ln, = plt.plot([], [], ".", label="AGV_position")
        # self._waypoint, = plt.plot([],[], "r")
        # # self.x_data, self.y_data = [X_pos_t] , [Y_pos_t]
        # self.yaw_t_data = yaw_t
        # self.yaw_h_data = yaw_h
        # self.bound, = plt.plot([], [], "k")
        # self.trailer_plot, = plt.plot([], [], "g")
        # self.head_plot, = plt.plot([], [], "g")
        # self.l_dock_, = plt.plot([], [], "m")
        # self.r_dock_, = plt.plot([], [], "m")
        # self.low_dock_, = plt.plot([], [], "m")

    def plot_init(self):
        self.ax.set_xlim(min(X_utm)-10, max(X_utm) + 50)
        self.ax.set_ylim(min(Y_utm)-10, max(Y_utm) + 10)
        self.ax.set_xlabel("X_utm(m)")
        self.ax.set_ylabel("Y_utm(m)")

        self.x_wp, self.y_wp = [], []
        self.ln, = plt.plot([], [], ".", label="CTT Ref Point")
        self._waypoint, = plt.plot([],[], "r")
        # self.x_data, self.y_data = [X_pos_t] , [Y_pos_t]
        self.bound, = plt.plot([], [], "k")
        self.trailer_plot, = plt.plot([], [], "g")
        self.head_plot, = plt.plot([], [], "g")
        self.l_dock_, = plt.plot([], [], "m")
        self.r_dock_, = plt.plot([], [], "m")
        self.low_dock_, = plt.plot([], [], "m")
        self.target_PSback, = plt.plot([],[], ".")
        self.target_PSdock, = plt.plot([],[], ".")
        self.xtodock, self.ytodock = 558098.5399999999, 9158001.32#min(X_utm) + 14.5, min(Y_utm) + 70
        self.start_to_dock, = plt.plot([],[], ".", label="start point")

        self.ax.legend()
        # self.axis("equal")
        # return self.ln

    def Trailer_(self, X_pos, Y_pos, yaw_): #(x position, y position, yaw angle, length, width)
        # Create polygon coordinates
        transform_coor = [[X_pos-self.l_t/2*np.cos(yaw_)+self.w_t/2*np.sin(yaw_),Y_pos-self.l_t/2*np.sin(yaw_)-self.w_t/2*np.cos(yaw_)],
                [X_pos-self.l_t/2*np.cos(yaw_)-self.w_t/2*np.sin(yaw_),Y_pos-self.l_t/2*np.sin(yaw_)+self.w_t/2*np.cos(yaw_)],
                [X_pos+self.l_t/2*np.cos(yaw_)-self.w_t/2*np.sin(yaw_),Y_pos+self.l_t/2*np.sin(yaw_)+self.w_t/2*np.cos(yaw_)],
                [X_pos+self.l_t/2*np.cos(yaw_)+self.w_t/2*np.sin(yaw_),Y_pos+self.l_t/2*np.sin(yaw_)-self.w_t/2*np.cos(yaw_)]]
        return transform_coor  

    def head_(self, X_pos_t, Y_pos_t, yaw_t, yaw_h): 
        # create head center point
        h_center = [X_pos_t + (self.l_t/2-0.755)*np.cos(yaw_t), Y_pos_t + (self.l_t/2-0.755)*np.sin(yaw_t)]
        h_transform = [[h_center[0]-self.l_h/2*np.cos(yaw_h)+self.w_h/2*np.sin(yaw_h),h_center[1]-self.l_h/2*np.sin(yaw_h)-self.w_h/2*np.cos(yaw_h)],
                [h_center[0]-self.l_h/2*np.cos(yaw_h)-self.w_h/2*np.sin(yaw_h),h_center[1]-self.l_h/2*np.sin(yaw_h)+self.w_h/2*np.cos(yaw_h)],
                [h_center[0]+self.l_h/2*np.cos(yaw_h)-self.w_h/2*np.sin(yaw_h),h_center[1]+self.l_h/2*np.sin(yaw_h)+self.w_h/2*np.cos(yaw_h)],
                [h_center[0]+self.l_h/2*np.cos(yaw_h)+self.w_h/2*np.sin(yaw_h),h_center[1]+self.l_h/2*np.sin(yaw_h)-self.w_h/2*np.cos(yaw_h)]]
        return h_transform                
    
    def dock_(self, X_pos, Y_pos, yaw_,l_, w_): #(x position, y position, yaw angle, length, width)
        # Create polygon coordinates
        transform_coor = [[X_pos-l_/2*np.cos(yaw_)+w_/2*np.sin(yaw_),Y_pos-l_/2*np.sin(yaw_)-w_/2*np.cos(yaw_)],
                [X_pos-l_/2*np.cos(yaw_)-w_/2*np.sin(yaw_),Y_pos-l_/2*np.sin(yaw_)+w_/2*np.cos(yaw_)],
                [X_pos+l_/2*np.cos(yaw_)-w_/2*np.sin(yaw_),Y_pos+l_/2*np.sin(yaw_)+w_/2*np.cos(yaw_)],
                [X_pos+l_/2*np.cos(yaw_)+w_/2*np.sin(yaw_),Y_pos+l_/2*np.sin(yaw_)-w_/2*np.cos(yaw_)]]
        return transform_coor  

    def sub_data_conv(self,msg):
        try:
            self.X_now = msg.pose.covariance[4]
            self.Y_now = msg.pose.covariance[5]
            self.yaw_now = msg.pose.covariance[7]
            self.yaw_head = msg.pose.covariance[8]
        except:
            print("No Data")    
    def callback_HMI(self,msg):
        try:
            self.control_mode = msg.pose.covariance[0]
        except:
            print("No Data")
    def update_plot(self, frame):
        # self.ax.set_xlim(min(self.x_data)-5, max(self.x_data)+5)
        # check if the waypoint visualization is needed
        if self.control_mode == 3: 
            # show the waypoints
            # self._waypoints, = self._dynamic_ax.plot([], [], "r", label="waypoints")
            self._waypoint.set_data(self.waypoints[:,0],self.waypoints[:,1])
            self._waypoint.figure.canvas.draw()
        elif self.control_mode == 4: 
            # self._target_PS, = self._dynamic_ax.plot([], [], ".", label="point stab target")
            self.target_PSback.set_data(PSbackX, PSbackY)
            self.target_PSback.figure.canvas.draw()
        elif self.control_mode == 5:
            # self._target_dock, = self._dynamic_ax.plot([], [], ".", label="docking target")
            self.target_PSdock.set_data(PSdockX, PSdockY)
            self.target_PSdock.figure.canvas.draw()
        else: 
            # clear the plotting area
            self.ax.cla()
            self.plot_init()
            # self._waypoints.cla()
            # pass

        self.ln.set_data(self.X_now, self.Y_now)
        self.bound.set_data(X_utm, Y_utm)
        AV_trailer = self.Trailer_(self.X_now + 11.92/2 * np.cos(self.yaw_now)\
            , self.Y_now + 11.92/2 * np.sin(self.yaw_now), self.yaw_now)
        x3,y3 = Polygon(AV_trailer).exterior.xy
        self.trailer_plot.set_data(x3,y3)
        AV_head = self.head_(self.X_now + 11.92/2 * np.cos(self.yaw_now)\
            , self.Y_now + 11.92/2 * np.sin(self.yaw_now), self.yaw_now, self.yaw_head)
        x4,y4 = Polygon(AV_head).exterior.xy
        self.head_plot.set_data(x4,y4)
        # create docking station
        l_dock_box = self.dock_(l_dock_[0],l_dock_[1],l_dock_[2],l_dock_[3],l_dock_[4])
        x5,y5 = Polygon(l_dock_box).exterior.xy
        self.l_dock_.set_data(x5,y5)
        r_dock_box = self.dock_(r_dock_[0],r_dock_[1],r_dock_[2],r_dock_[3],r_dock_[4])
        x6,y6 = Polygon(r_dock_box).exterior.xy
        self.r_dock_.set_data(x6,y6)
        low_dock_box = self.dock_(low_dock_[0],low_dock_[1],low_dock_[2],low_dock_[3],low_dock_[4])
        x7,y7 = Polygon(low_dock_box).exterior.xy
        self.low_dock_.set_data(x7,y7)

        self.start_to_dock.set_data(self.xtodock, self.ytodock)

        return self.ln, self.trailer_plot, self.head_plot, self.bound,\
             self.l_dock_, self.r_dock_, self.low_dock_, self.start_to_dock

rospy.init_node('visualization')
vis = Visualiser()
rospy.Subscriber('/data_conv', PoseWithCovarianceStamped, vis.sub_data_conv)
rospy.Subscriber('/idc_lamp', PoseWithCovarianceStamped, vis.callback_HMI)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.axis("square")
plt.show(block=True) 