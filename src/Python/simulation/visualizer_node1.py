import matplotlib.pyplot as plt
import rospy
import CTT_Sim_Func as CTT
# import tf
# from nav_msgs.msg import Odometry
from thesis.msg import Game_Theory_Logger
from thesis.msg import State_Estimator
# from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
from shapely.geometry import Polygon

# UTM coordinates
X_utm = [558103.34,558082.38, 558082.34, 558082.34, 558103.22, 558103.30, 558103.34]
Y_utm = [9158033.03, 9158033.05, 9157996.57, 9157931.34, 9157931.32, 9157996.55, 9158033.03]
# Resized coordinates (move the baseline to zero)
X_res = np.array(X_utm[0:]) - min(X_utm)
Y_res = np.array(Y_utm[0:]) - min(Y_utm)
# initial position
X_pos_t = 17
Y_pos_t = 21
yaw_t = 0.5*3.14
yaw_h = 0.5*3.14
# target position
X_target = 11.5
Y_target = 70
# vehicle parameters
l_t = 13.75
w_t = 2.682
l_h = 3.533
w_h = 2.682
# figure
width = 40
height = 40
# docking spec (x,y,orientation,length,width)
# left docking
l_dock_ = [3, 7, 0.5*np.pi, 10, 0.2]
# right docking
r_dock_ = [9, 7, 0.5*np.pi, 10, 0.2]
# lower docking
low_dock_ = [6, 2, np.pi, 6, 0.2]

class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(width,height))
        self.ln, = plt.plot([], [], ".", label="AGV_trajectory")
        self.waypoint, = plt.plot([],[], "--", label="AGV_Waypoint")
        self.x_data, self.y_data = [X_pos_t] , [Y_pos_t]
        self.yaw_t_data = yaw_t
        self.yaw_h_data = yaw_h
        self.bound, = plt.plot([], [], "k")
        self.trailer_plot, = plt.plot([], [], "g")
        self.head_plot, = plt.plot([], [], "g")
        self.init_pos, = plt.plot([X_pos_t],[Y_pos_t], ".", c="b", label="Start Position")
        self.end_pos, = plt.plot([X_target],[Y_target], ".", c="r", label="Target Position")
        self.x_wp, self.y_wp = [], []
        self.l_dock_, = plt.plot([], [], "m")
        self.r_dock_, = plt.plot([], [], "m")
        self.low_dock_, = plt.plot([], [], "m")

    def plot_init(self):
        self.ax.set_xlim(-40, 80)
        self.ax.set_ylim(-10, 110)
        self.ax.set_xlabel("X(m)")
        self.ax.set_ylabel("Y(m)")
        self.ax.legend()
        # self.axis("equal")
        # return self.ln
    
    def pos_callback(self, msg):
        self.x_data.append(msg.actual_x_av)
        self.y_data.append(msg.actual_y_av)
        self.yaw_t_data = msg.actual_yaw_av
        self.yaw_h_data = msg.actual_steer_av + self.yaw_t_data
        self.x_wp = msg.path_x_av 
        self.y_wp = msg.path_y_av
    
    def update_plot(self, frame):
        # self.ax.set_xlim(min(self.x_data)-5, max(self.x_data)+5)
        self.ln.set_data(self.x_data, self.y_data)
        self.waypoint.set_data(self.x_wp, self.y_wp)
        self.bound.set_data(X_res, Y_res)
        AV_trailer = CTT.Trailer_(self.x_data[-1], self.y_data[-1], self.yaw_t_data,l_t, w_t)
        x3,y3 = Polygon(AV_trailer).exterior.xy
        self.trailer_plot.set_data(x3,y3)
        AV_head = CTT.head_(self.x_data[-1], self.y_data[-1], self.yaw_t_data, l_t, w_t, l_h, w_h, self.yaw_h_data)
        x4,y4 = Polygon(AV_head).exterior.xy
        self.head_plot.set_data(x4,y4)
        # create docking station
        l_dock_box = CTT.Trailer_(l_dock_[0],l_dock_[1],l_dock_[2],l_dock_[3],l_dock_[4])
        x5,y5 = Polygon(l_dock_box).exterior.xy
        self.l_dock_.set_data(x5,y5)
        r_dock_box = CTT.Trailer_(r_dock_[0],r_dock_[1],r_dock_[2],r_dock_[3],r_dock_[4])
        x6,y6 = Polygon(r_dock_box).exterior.xy
        self.r_dock_.set_data(x6,y6)
        low_dock_box = CTT.Trailer_(low_dock_[0],low_dock_[1],low_dock_[2],low_dock_[3],low_dock_[4])
        x7,y7 = Polygon(low_dock_box).exterior.xy
        self.low_dock_.set_data(x7,y7)

        print("steering data: "+str(self.yaw_h_data))

        return self.ln, self.waypoint, self.trailer_plot, self.head_plot, self.bound,\
             self.init_pos, self.end_pos, self.l_dock_, self.r_dock_, self.low_dock_



rospy.init_node('visualization')
vis = Visualiser()
sub = rospy.Subscriber('/CTT_movement', Game_Theory_Logger, vis.pos_callback)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.axis("square")
plt.show(block=True) 