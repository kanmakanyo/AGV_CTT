#! /usr/bin/env python3
# CINOVASI - HAM MAF
# Node name : data_conv
# Node number : 4
# This node is used to convert every raw data to clean data that ready to be used by another node

#-----------------------------------------------------------------------------#
# Function/library declaration
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import scipy
import scipy.linalg
from agv_ctt.msg import LocalizationControllerResultMessage0502
from agv_ctt.msg import OdometryMessage0105
#-----------------------------------------------------------------------------#
# create header for publisher node
# Message publisher initialization
rospy.init_node('data_converter')
freq = 20 # Hz
pub = rospy.Publisher('/data_conv', PoseWithCovarianceStamped, queue_size=1)
rate = rospy.Rate(freq) # Hz
pub_msg = PoseWithCovarianceStamped()
pub_msg.header.frame_id = 'data_converter_node'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()
#-----------------------------------------------------------------------------#
# Initialization parameter
x_est, y_est, yaw_est= 0.0, 0.0, 0.0
YS31, YS32 = 0, 0
beta = 0.7
filter_flag = 0
lever_mode = 0
control_mode = 0
reference_voltage = rospy.get_param('~Reference_Voltage', 20) #check!
volt_f_steer = 0
volt_f_throttle = 0
deg_steer = 0
rad_steer = 0
x_pos, y_pos, yaw_trailer = 0., 0., 0. 
imu_x, imu_y, imu_z, imu_w = 0.0, 0.0, 0.0, 0.0
imu_roll, imu_pitch, imu_yaw = 0.0, 0.0, 0.0
seq_front = 0
seq_rear = 0
sensor_data = {
    'x_front': 685552.00531,
    'y_front': 9204398.54478, 
    'x_rear': 685552.00531, 
    'y_rear': 9204398.54478 ,
    'x_crw': 0.,
    'y_crw': 0.,
    'x_rear_bef': 0.,
    'y_rear_bef': 0., 
    'x_gnssprev':685552.00531,
    'y_gnssprev':9204398.54478,
    'yaw_prev':0.799214
}
data_bef = {
    'xf_bef':0.,
    'yf_bef':0.,
    'xr_bef':0.,
    'yr_bef':0.
}
temp_msg = {
    'x_est': 685552.00531,#0.,xc
    'y_est': 9204398.54478,#0.,
    'v_est': 0.,
    'yaw_est': 0.799214,#0.,
    'yaw_trailer': 0.799214,#0.
    'yaw_trailer_bef': 0.799214,#0.
    'yaw_head': 0.799214,
    'yaw_gnss': 0.799214, #---
    'x_est_nlo': 685552.00531,
    'y_est_nlo': 9204398.54478,
    'teta_est_nlo': 0.799214,
    'x_est_enc': 685552.00531,
    'y_est_enc': 9204398.54478,
    'teta_est_enc': 0.799214,
    'yaw_trailer':0.799214
}

state_est = {
    'x': 0.,
    'y': 0.,
    'teta': 0.,
    'check_alg' : 0.
}

# parameter covariance of unscented kalman filter
proc = np.array([0.3, 0.2, 0.3])
Q = np.diag(proc)
meas = np.array([0.5, 0.5, 0.5])
R = np.diag(meas)
cov = np.array([0.09, 0.09, 0.09])
P_k = np.diag(cov)

Q_y = 0.3
R_y = 1
P_k_y = 0.09

# parameter sigma point unscented kalman filter
l = 11.92
h_est = 0.05   #dt
h = 0.05       #dt 
kx = 10   
ky = 10
kteta = 0.5
n = 3
alpha = 10**(-3)
kappa = 0
beta = 2
lamda = (n+kappa)*alpha**2 - n
lamda_y = (1+kappa)*alpha**2 - 1
i = 0
#-----------------------------------------------------------------------------#
# other function 
def to_euler(x, y, z, w): 
    roll = np.arctan2(2.0 * (y*x + w*z), w**2 + x**2 - y**2 - z**2)
    pitch = np.arcsin(-2.0 * (x*z - w*y)/(w**2 + x**2 + y**2 + z**2))
    yaw = np.arctan2(2.0 * (w*z + x*y), w**2 - x**2 - y**2 + z**2)
    return np.array([roll, pitch, yaw])

# def estimator(x,y,teta,v,delta): # UKF
def estimator(): # UKF
    #x = x_rear
    #y = y_rear
    #teta = yaw_gnss
    #v = v_est 
    #delta = rad_steer
    global sensor_data
    global temp_msg
    global state_est
    global Q, R, P_k
    global Q_y, R_y, P_k_y
    global l, h, kx, ky, kteta, h_est
    global n, alpha, kappa, beta, lamda, lamda_y
    global xc, yc, tetac

    # get data sensor
    # sensor_data['x_rear'] = x
    # sensor_data['y_rear'] = y
    # temp_msg['yaw_gnss'] = teta
    # sensor_data['delta'] = -delta

    # speed direction is defined based on the lever cond
    if YS31 and not YS32: #reverse
        v = -temp_msg['v_est']
    else: 
        v = temp_msg['v_est']

    # temp_msg['yaw_trailer'] = sensor_data['yaw_gnss'] + 1.6491
    # temp_msg['yaw_trailer'] %= 2 * np.pi

    # estimate yaw

    Wm1 = 0.5/(1+lamda_y)
    Wm = ([lamda_y/(1+lamda_y), Wm1, Wm1])
    Wc1 = lamda_y / (1+lamda_y) + (1+beta-alpha**2)
    Wc = ([Wc1, Wm1, Wm1])

    x_k_y = temp_msg['teta_est_nlo']
    ymeas_y = temp_msg['yaw_trailer']

    P_k_y = ((1+lamda_y)*P_k_y)

    C_y = np.sqrt(P_k_y)
    X0_y = x_k_y
    X_k_y = np.zeros((1,3))
    X_k_y[:,0]= x_k_y
    X_k_y[:,1]= (X0_y+C_y)
    X_k_y[:,2]= (X0_y-C_y)

    X_kest_y = np.zeros((1,1))
    X_Aug_y = np.zeros((1,3))

    for i in range(3):
        tetan = X_k_y[0,i]

        tetae = (sensor_data['yaw_prev']-tetan)

        tetac = tetan + h_est*kteta*tetae

        X_Aug_y[:,i] = [tetac + h_est*(v/l)*np.tan(rad_steer)]
        X_kest_y = X_kest_y + Wm[i]*np.transpose([X_Aug_y[:,i]])

    X_kestt_y = np.transpose(X_kest_y)
    X_kesta_y = np.transpose([X_kestt_y, X_kestt_y, X_kestt_y])

    X_kesta_y = np.array(X_kesta_y)
    X_kesta_y.shape
    X_kesta_y = np.squeeze(X_kesta_y)

    X_er_y = X_Aug_y - X_kesta_y

    P_k__y = X_er_y @ np.diag(Wc) @ (np.transpose(X_er_y)) + Q_y

    Y_kest_y = np.zeros((1,1))
    Y_Aug_y = np.zeros((1,3))

    for i in range(3):
        Y_Aug_y[:,i] = [X_Aug_y[0,i]]
  
        Y_kest_y = Y_kest_y + Wm[i]*np.transpose([X_Aug_y[:,i]])

    Y_kestt_y = np.transpose(Y_kest_y)
    Y_kesta_y = np.transpose([Y_kestt_y, Y_kestt_y, Y_kestt_y])

    Y_kesta_y = np.array(Y_kesta_y)
    Y_kesta_y.shape
    Y_kesta_y = np.squeeze(Y_kesta_y)

    Y_er_y = Y_Aug_y - Y_kesta_y

    Py_k_y = Y_er_y @ np.diag(Wc) @ (np.transpose(Y_er_y)) + R_y

    Pxy_k_y = X_er_y @ np.diag(Wc) @ (np.transpose(Y_er_y))

    K_k_y =  Pxy_k_y/Py_k_y

    x_kx_y = X_kest_y + K_k_y @ (ymeas_y-Y_kest_y)

    P_k_y = P_k__y - K_k_y @ Py_k_y @ ((K_k_y))

    temp_msg['teta_est_nlo'] = x_kx_y
    temp_msg['teta_est_nlo'] = float(temp_msg['teta_est_nlo'])

    # estimate position

    Wm1 = 0.5/(n+lamda)
    Wm = ([lamda/(n+lamda), Wm1, Wm1, Wm1, Wm1])
    Wc1 = lamda / (n+lamda) + (1+beta-alpha**2)
    Wc = ([Wc1, Wm1, Wm1, Wm1, Wm1])

    temp_msg['x_est'] = sensor_data['x_rear'] + 0.54 * np.sin(temp_msg['teta_est_nlo'])-\
         0.7 * np.cos(temp_msg['teta_est_nlo']) - 11.92 * np.cos(temp_msg['teta_est_nlo'])
    temp_msg['y_est'] = sensor_data['y_rear'] - 0.54 * np.cos(temp_msg['teta_est_nlo'])-\
         0.7 * np.sin(temp_msg['teta_est_nlo']) - 11.92 * np.sin(temp_msg['teta_est_nlo'])

    # temp_msg['y_est'] = (temp_msg['y_t'])
    # temp_msg['x_est'] = (temp_msg['x_t'])

    ymeas = [[temp_msg['x_est']], [temp_msg['y_est']]]

    x_k = [temp_msg['x_est_nlo'], temp_msg['y_est_nlo']]  

    P_k = np.transpose((2+lamda)*P_k)

    C = np.array(scipy.linalg.cholesky(P_k, lower=True))
    X0 = np.transpose([x_k, x_k])
    X_k = np.zeros((2,5))
    X_k[:,0]= np.transpose(x_k)
    X_k[:,1:3]= (X0+C)
    X_k[:,3:5]= (X0-C)

    X_kest = np.zeros((2,1))
    X_Aug = np.zeros((2,5))

    for i in range(5):
        xn = X_k[0,i]
        yn = X_k[1,i]

        xe = (sensor_data['x_gnssprev']-(xn))
        ye = (sensor_data['y_gnssprev']-(yn))

        xc = xn + h_est*kx*xe
        yc = yn + h_est*ky*ye

        X_Aug[:,i] =  [xc + h_est*v*np.cos(temp_msg['teta_est_nlo']), yc + h_est*v*np.sin(temp_msg['teta_est_nlo'])]
        X_kest = X_kest + Wm[i]*np.transpose([X_Aug[:,i]])

    X_kestt = np.transpose(X_kest)
    X_kesta = np.transpose([X_kestt, X_kestt, X_kestt, X_kestt, X_kestt])
    X_kesta = np.array(X_kesta)
    X_kesta.shape
    X_kesta = np.squeeze(X_kesta)

    X_er = X_Aug - X_kesta

    P_k_ = X_er @ np.diag(Wc) @ (np.transpose(X_er)) + Q

    Y_kest = np.zeros((2,1))
    Y_Aug = np.zeros((2,5))

    for i in range(5):
        Y_Aug[:,i] = [X_Aug[0,i], X_Aug[1,i]]
  
        Y_kest = Y_kest + Wm[i]*np.transpose([X_Aug[:,i]])

    Y_kestt = np.transpose(Y_kest)
    Y_kesta = np.transpose([Y_kestt, Y_kestt, Y_kestt, Y_kestt, Y_kestt])

    Y_kesta = np.array(Y_kesta)
    Y_kesta.shape
    Y_kesta = np.squeeze(Y_kesta)

    Y_er = Y_Aug - Y_kesta

    Py_k = Y_er @ np.diag(Wc) @ (np.transpose(Y_er)) + R

    Pxy_k = X_er @ np.diag(Wc) @ (np.transpose(Y_er))

    K_k = (np.linalg.inv(Py_k)) @ Pxy_k

    x_kx = X_kest + K_k @ (ymeas-Y_kest)

    P_k = P_k_ - K_k @ Py_k @ (np.transpose(K_k))

    temp_msg['x_est_nlo'] = float(x_kx[0])
    temp_msg['y_est_nlo'] = float(x_kx[1])
    sensor_data['x_gnssprev'] = float(temp_msg['x_est_nlo'])
    sensor_data['y_gnssprev'] = float(temp_msg['y_est_nlo'])
    sensor_data['yaw_prev'] = float(temp_msg['teta_est_nlo'])

    state_est['check_alg'] = 1

    # result of estimation
    # xc = temp_msg['x_est_nlo']
    # yc = temp_msg['y_est_nlo']
    # tetac = temp_msg['teta_est_nlo']

    temp_msg['teta_est_nlo'] %= 2 * np.pi
    # tetac %= 2 * np.pi

# def estimator(x,y,teta,v,delta): # UKF
def estimator_lidar(): # UKF
    #x = x_rear
    #y = y_rear
    #teta = yaw_gnss
    #v = v_est 
    #delta = rad_steer
    global sensor_data
    global temp_msg
    global state_est
    global Q, R, P_k
    global Q_y, R_y, P_k_y
    global l, h, kx, ky, kteta, h_est
    global n, alpha, kappa, beta, lamda, lamda_y
    global xc, yc, tetac

    # get data sensor
    # sensor_data['x_rear'] = x
    # sensor_data['y_rear'] = y
    # temp_msg['yaw_gnss'] = teta
    # sensor_data['delta'] = -delta

    # speed direction is defined based on the lever cond
    if YS31 and not YS32: #reverse
        v = -temp_msg['v_est']
    else: 
        v = temp_msg['v_est']

    # temp_msg['yaw_trailer'] = sensor_data['yaw_gnss'] + 1.6491
    # temp_msg['yaw_trailer'] %= 2 * np.pi

    # estimate position

    Wm1 = 0.5/(3+lamda)
    Wm = ([lamda/(3+lamda), Wm1, Wm1, Wm1, Wm1, Wm1])
    Wc1 = lamda / (3+lamda) + (1+beta-alpha**2)
    Wc = ([Wc1, Wm1, Wm1, Wm1, Wm1, Wm1, Wm1])

    # temp_msg['y_est'] = (temp_msg['y_t'])
    # temp_msg['x_est'] = (temp_msg['x_t'])

    ymeas = [[x_est], [y_est],[yaw_est]]

    x_k = [temp_msg['x_est_nlo'], temp_msg['y_est_nlo'], temp_msg['teta_est_nlo']]  

    P_k = np.transpose((3+lamda)*P_k)

    C = np.array(scipy.linalg.cholesky(P_k, lower=True))
    X0 = np.transpose([x_k, x_k, x_k])
    X_k = np.zeros((3,7))
    X_k[:,0]= np.transpose(x_k)
    X_k[:,1:4]= (X0+C)
    X_k[:,4:7]= (X0-C)

    X_kest = np.zeros((3,1))
    X_Aug = np.zeros((3,7))

    for i in range(7):
        xn = X_k[0,i]
        yn = X_k[1,i]
        tetan = X_k[2,i]

        xe = (sensor_data['x_gnssprev']-(xn))
        ye = (sensor_data['y_gnssprev']-(yn))
        tetae = (sensor_data['yaw_prev']-(tetan))

        xc = xn + h_est*kx*xe
        yc = yn + h_est*ky*ye
        tetac = tetan + h_est*kteta*tetae

        X_Aug[:,i] =  [xc + h_est*v*np.cos(temp_msg['teta_est_nlo']), yc + h_est*v*np.sin(temp_msg['teta_est_nlo']),tetac + h_est*(v/l)*np.tan(rad_steer)]
        X_kest = X_kest + Wm[i]*np.transpose([X_Aug[:,i]])

    X_kestt = np.transpose(X_kest)
    X_kesta = np.transpose([X_kestt, X_kestt, X_kestt, X_kestt, X_kestt, X_kestt, X_kestt])
    X_kesta = np.array(X_kesta)
    X_kesta.shape
    X_kesta = np.squeeze(X_kesta)

    X_er = X_Aug - X_kesta

    P_k_ = X_er @ np.diag(Wc) @ (np.transpose(X_er)) + Q

    Y_kest = np.zeros((3,1))
    Y_Aug = np.zeros((3,7))

    for i in range(7):
        Y_Aug[:,i] = [X_Aug[0,i], X_Aug[1,i], X_Aug[2,i]]
  
        Y_kest = Y_kest + Wm[i]*np.transpose([X_Aug[:,i]])

    Y_kestt = np.transpose(Y_kest)
    Y_kesta = np.transpose([Y_kestt, Y_kestt, Y_kestt, Y_kestt, Y_kestt, Y_kestt, Y_kestt])

    Y_kesta = np.array(Y_kesta)
    Y_kesta.shape
    Y_kesta = np.squeeze(Y_kesta)

    Y_er = Y_Aug - Y_kesta

    Py_k = Y_er @ np.diag(Wc) @ (np.transpose(Y_er)) + R

    Pxy_k = X_er @ np.diag(Wc) @ (np.transpose(Y_er))

    K_k = (np.linalg.inv(Py_k)) @ Pxy_k

    x_kx = X_kest + K_k @ (ymeas-Y_kest)

    P_k = P_k_ - K_k @ Py_k @ (np.transpose(K_k))

    temp_msg['x_est_nlo'] = float(x_kx[0])
    temp_msg['y_est_nlo'] = float(x_kx[1])
    temp_msg['teta_est_nlo'] = float(x_kx[2])
    sensor_data['x_gnssprev'] = float(temp_msg['x_est_nlo'])
    sensor_data['y_gnssprev'] = float(temp_msg['y_est_nlo'])
    sensor_data['yaw_prev'] = float(temp_msg['teta_est_nlo'])

    state_est['check_alg'] = 1

    # result of estimation
    # xc = temp_msg['x_est_nlo']
    # yc = temp_msg['y_est_nlo']
    # tetac = temp_msg['teta_est_nlo']

    temp_msg['teta_est_nlo'] %= 2 * np.pi
    # tetac %= 2 * np.pi

#-----------------------------------------------------------------------------#
# Callback function -> CHECK!
def callback_control(msg):
    # Get data from control_algorithmm node
    global control_mode, lever_mode, filter_flag
    control_mode = msg.pose.covariance[4]
    lever_mode = msg.pose.covariance[18]
    filter_flag = msg.pose.covariance[23]

# Callback function
def callback_dbwsub(msg):
    # Get data adc from PLC (ADC) --> Feedback (sensor)
    global volt_f_steer, volt_f_throttle, rad_steer, deg_steer, temp_msg
    global YS31, YS32
    autonomous_switch = msg.pose.covariance[16]
    cal_steer = msg.pose.covariance[1]      # Steer       
    cal_throttle = msg.pose.covariance[0]   # Throttle? to be confirmed
    # Conversi data from PLC (ADC) to volt
    # re-check the voltage value and the offset value
    volt_f_steer = ((cal_steer / 65535)* reference_voltage) #in volt (CHECK!)
    deg_steer = ((volt_f_steer - 2.6603)/0.013) -1 #- 6.5 #1.6 #+ 1.6 #in degree
    # if control_mode == 1:
    #     deg_steer = deg_steer + 1
    YS31 = msg.pose.covariance[18]
    YS32 = msg.pose.covariance[19]

    if YS31 and not YS32: #reverse
        deg_steer = deg_steer - 4#- 3.4
    
    rad_steer = np.radians(deg_steer) #in radians
    volt_f_throttle = ((cal_throttle / 65535)* 5) #out: (0-5)V value

def gnssFrontCallback(msg):
    # Get position data from the front GNSS
    global temp_msg, seq_front
    global sensor_data

    sensor_data['x_front'] = msg.pose.pose.position.x
    sensor_data['y_front'] = msg.pose.pose.position.y
    # seq_front = msg.header.seq

    # delta_t = 0.2 #ms (5 Hz)
    # if seq_front < 2: 
    #     data_bef['xf_bef'] = sensor_data['x_front']
    #     data_bef['yf_bef'] = sensor_data['y_front']
    #     temp_msg['v_est'] = 0.
    #     temp_msg['yaw_gnss'] = 0.76 #- 1.6491
    #     temp_msg['yaw_head'] = temp_msg['yaw_trailer'] + rad_steer
    # else: 
    #     dydxgnss = np.arctan2(sensor_data['y_front']-data_bef['yf_bef'],
    #                                                     sensor_data['x_front']-data_bef['xf_bef'])
    #     temp_msg['v_est'] = np.sqrt((data_bef['xf_bef']-sensor_data['x_front'])**2 + (data_bef['yf_bef']-sensor_data['y_front'])**2) / delta_t
    #     if temp_msg['v_est'] > 0.07: 
    #         temp_msg['yaw_gnss'] = dydxgnss

    #     if YS31 and not YS32: #reverse
    #         temp_msg['yaw_gnss'] = dydxgnss + np.pi

    #     temp_msg['yaw_trailer'] = temp_msg['yaw_gnss'] #+ 1.6491#+ np.pi/2
    #     temp_msg['yaw_head'] = temp_msg['yaw_trailer'] + rad_steer
    #     data_bef['xf_bef'] = sensor_data['x_front']
    #     data_bef['yf_bef'] = sensor_data['y_front']

    # calculate head yaw
    delta_t = 0.2 #ms (5 Hz)
    temp_msg['v_est'] = np.sqrt((data_bef['xf_bef']-sensor_data['x_front'])**2 + (data_bef['yf_bef']-sensor_data['y_front'])**2) / delta_t
    data_bef['xf_bef'] = sensor_data['x_front']
    data_bef['yf_bef'] = sensor_data['y_front']

    # if seq_front == seq_rear: 
    #     temp_msg['yaw_gnss'] = np.arctan2(sensor_data['y_front']-sensor_data['y_rear'],
    #                                                     sensor_data['x_front']-sensor_data['x_rear'])
    #     temp_msg['yaw_trailer'] = temp_msg['yaw_gnss'] + 1.6491#+ np.pi/2
    #     temp_msg['yaw_head'] = temp_msg['yaw_trailer'] + rad_steer
    
    # if filter_flag: 
    #     temp_msg['yaw_trailer'] = beta * temp_msg['yaw_trailer_bef'] + (1-beta)*temp_msg['yaw_trailer']
    # else: 
    #     temp_msg['yaw_trailer_bef'] = temp_msg['yaw_trailer']

    # from HAM notes
    # coordinates transformation
    # X_cgnss = sensor_data['x_rear'] + 0.54 * np.cos(temp_msg['yaw_gnss'])
    # Y_cgnss = sensor_data['y_rear'] + 0.54 * np.sin(temp_msg['yaw_gnss'])
    # X_cfw = X_cgnss + 0.7 * np.cos(temp_msg['yaw_trailer']-(np.pi))
    # Y_cfw = Y_cgnss + 0.7 * np.sin(temp_msg['yaw_trailer']-(np.pi))
    # X_crw = X_cfw + 11.92 * np.cos(temp_msg['yaw_trailer'] - np.pi)
    # Y_crw = Y_cfw + 11.92 * np.sin(temp_msg['yaw_trailer'] - np.pi)

    # sensor_data['x_crw'] = X_crw
    # sensor_data['y_crw'] = Y_crw
    # temp_msg['x_est'] = X_crw
    # temp_msg['y_est'] = Y_crw

    # X_cgnss = sensor_data['x_front'] + 0.54 * np.cos(temp_msg['yaw_gnss'])
    # Y_cgnss = sensor_data['y_front'] + 0.54 * np.sin(temp_msg['yaw_gnss'])
    # X_cfw = X_cgnss + 0.7 * np.cos(temp_msg['yaw_trailer']-(np.pi))
    # Y_cfw = Y_cgnss + 0.7 * np.sin(temp_msg['yaw_trailer']-(np.pi))
    # X_crw = X_cfw + 11.92 * np.cos(temp_msg['yaw_trailer'] - np.pi)
    # Y_crw = Y_cfw + 11.92 * np.sin(temp_msg['yaw_trailer'] - np.pi)

    # sensor_data['x_crw'] = X_crw
    # sensor_data['y_crw'] = Y_crw

    # temp_msg['x_est'] = X_crw
    # temp_msg['y_est'] = Y_crw

    # estimator()

    seq_front +=1

    if seq_front > 100: 
        seq_front = 0

def gnssRearCallback(msg):
    # Get position data from the front GNSS
    global temp_msg, seq_rear
    global sensor_data
    sensor_data['x_rear'] = msg.pose.pose.position.x
    sensor_data['y_rear'] = msg.pose.pose.position.y

    # seq_rear = msg.header.seq
    seq_rear +=1

    delta_t = 0.2 #ms (5 Hz)

    if seq_front == seq_rear: 
        print("inside")
        temp_msg['yaw_gnss'] = np.arctan2(sensor_data['y_front']-sensor_data['y_rear'],
                                                        sensor_data['x_front']-sensor_data['x_rear'])
        temp_msg['yaw_trailer'] = temp_msg['yaw_gnss'] + 1.6491#+ np.pi/2
        temp_msg['yaw_head'] = temp_msg['yaw_trailer'] + rad_steer
    
    # temp_msg['yaw_gnss'] = np.arctan2(sensor_data['y_front']-sensor_data['y_rear'],
    #                                                 sensor_data['x_front']-sensor_data['x_rear'])
    # temp_msg['yaw_trailer'] = temp_msg['yaw_gnss'] + 1.6491#+ np.pi/2
    # temp_msg['yaw_head'] = temp_msg['yaw_trailer'] + rad_steer


    if filter_flag: 
        temp_msg['yaw_trailer'] = beta * temp_msg['yaw_trailer_bef'] + (1-beta)*temp_msg['yaw_trailer']
    else: 
        temp_msg['yaw_trailer_bef'] = temp_msg['yaw_trailer']

    # from HAM notes
    # coordinates transformation
    X_cgnss = sensor_data['x_rear'] + 0.54 * np.cos(temp_msg['yaw_gnss'])
    Y_cgnss = sensor_data['y_rear'] + 0.54 * np.sin(temp_msg['yaw_gnss'])
    X_cfw = X_cgnss + 0.7 * np.cos(temp_msg['yaw_trailer']-(np.pi))
    Y_cfw = Y_cgnss + 0.7 * np.sin(temp_msg['yaw_trailer']-(np.pi))
    X_crw = X_cfw + 11.92 * np.cos(temp_msg['yaw_trailer'] - np.pi)
    Y_crw = Y_cfw + 11.92 * np.sin(temp_msg['yaw_trailer'] - np.pi)

    sensor_data['x_crw'] = X_crw
    sensor_data['y_crw'] = Y_crw

    temp_msg['x_est'] = X_crw
    temp_msg['y_est'] = Y_crw

    # estimator()
    # seq_rear +=
    
    if seq_rear > 100: 
        seq_rear = 0

def callback_imu(msg): 
    global imu_roll, imu_pitch, imu_yaw
    x = msg.orientation.x
    y = msg.orientation.y
    z = msg.orientation.z
    w = msg.orientation.w
    euler = to_euler(x,y,z,w)
    imu_roll = euler[0]
    imu_pitch = euler[1]
    imu_yaw = euler[2]
                
def callback_lidar_pos(msg):
    global x_est, y_est, yaw_est

    x_lidar = float(msg.x)/1000
    y_lidar = float(msg.y)/1000
    yaw_lidar = np.radians(float(msg.heading)/1000)

    yaw_est = yaw_lidar #+ np.pi
    x_est = x_lidar #+ 1 * np.cos(yaw_est) + (0) * np.sin(yaw_est)
    y_est = y_lidar

#-----------------------------------------------------------------------------#
# subscriber initialization
rospy.Subscriber('/dbw_pubsub', PoseWithCovarianceStamped, callback_dbwsub)
rospy.Subscriber('/utm', Odometry, gnssFrontCallback)
rospy.Subscriber('/utm2', Odometry, gnssRearCallback)
rospy.Subscriber('/imu/data', Imu, callback_imu)
rospy.Subscriber('/control_algorithm', PoseWithCovarianceStamped, callback_control)
rospy.Subscriber('/localizationcontroller/out/localizationcontroller_result_message_0502', \
    LocalizationControllerResultMessage0502, callback_lidar_pos)
#-----------------------------------------------------------------------------#
while not rospy.is_shutdown():
    # list the message to publish

    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.pose.covariance[0] = volt_f_steer #steering angle in volt
    pub_msg.pose.covariance[1] = deg_steer #steering angle in degree
    pub_msg.pose.covariance[2] = rad_steer #steering angle in radian
    pub_msg.pose.covariance[3] = volt_f_throttle #throttle input voltage
    pub_msg.pose.covariance[4] = x_est
    pub_msg.pose.covariance[5] = y_est 
    pub_msg.pose.covariance[7] = yaw_est
    pub_msg.pose.covariance[8] = yaw_est + rad_steer
    # pub_msg.pose.covariance[4] = temp_msg['x_est'] 
    # pub_msg.pose.covariance[5] = temp_msg['y_est']  
    # pub_msg.pose.covariance[7] = temp_msg['yaw_trailer']
    # pub_msg.pose.covariance[8] = temp_msg['yaw_trailer'] + rad_steer
    # pub_msg.pose.covariance[4] = temp_msg['x_est_nlo']
    # pub_msg.pose.covariance[5] = temp_msg['y_est_nlo'] 
    # pub_msg.pose.covariance[7] = temp_msg['teta_est_nlo']
    # pub_msg.pose.covariance[8] = temp_msg['teta_est_nlo'] + rad_steer
    pub_msg.pose.covariance[6] = temp_msg['v_est']
    pub_msg.pose.covariance[9] = np.degrees(imu_roll)
    pub_msg.pose.covariance[10] = np.degrees(imu_pitch)
    pub_msg.pose.covariance[11] = np.degrees(imu_yaw)
    pub_msg.pose.covariance[12] = temp_msg['yaw_gnss']
    pub_msg.pose.covariance[13] = temp_msg['x_est']
    pub_msg.pose.covariance[14] = temp_msg['y_est'] 
    pub_msg.pose.covariance[15] = temp_msg['yaw_trailer']
    # pub_msg.pose.covariance[13] = x_est
    # pub_msg.pose.covariance[14] = y_est
    # pub_msg.pose.covariance[15] = yaw_est
    pub_msg.pose.covariance[16] = temp_msg['x_est_nlo']
    pub_msg.pose.covariance[17] = temp_msg['y_est_nlo']
    pub_msg.pose.covariance[18] = temp_msg['teta_est_nlo']    
    # pub_msg.pose.covariance[16] = temp_msg['x_est']
    # pub_msg.pose.covariance[17] = temp_msg['y_est'] 
    # pub_msg.pose.covariance[18] = temp_msg['yaw_trailer'] 

    # print(temp_msg['x_est'],temp_msg['y_est'],temp_msg['yaw_trailer'])
    
    # print(temp_msg['yaw_gnss'])

    # Publish the message
    pub.publish(pub_msg)

    ### Wait until the next loop
    rate.sleep()
