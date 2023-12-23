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
lever_mode = 0
control_mode = 0
reference_voltage = rospy.get_param('~Reference_Voltage', 20) #check!
volt_f_steer = 0
volt_f_throttle = 0
deg_steer = 0
rad_steer = 0
imu_x, imu_y, imu_z, imu_w = 0.0, 0.0, 0.0, 0.0
imu_roll, imu_pitch, imu_yaw = 0.0, 0.0, 0.0
sensor_data = {
    'x_front': 0.,
    'y_front': 0., 
    'x_rear': 0., 
    'y_rear': 0.,
    'x_crw': 0.,
    'y_crw': 0.
}
data_bef = {
    'xf_bef':0.,
    'yf_bef':0.,
    'xr_bef':0.,
    'yr_bef':0.
}
temp_msg = {
    'x_est': 558093.84,#0.,xc
    'y_est': 9158001.32,#0.,
    'v_est': 0.,
    'yaw_est': 1.5707,#0.,
    'yaw_trailer': 1.5707,#0.
    'yaw_head': 1.5707
}
#-----------------------------------------------------------------------------#
# other function 
def to_euler(x, y, z, w): 
    roll = np.arctan2(2.0 * (y*x + w*z), w**2 + x**2 - y**2 - z**2)
    pitch = np.arcsin(-2.0 * (x*z - w*y)/(w**2 + x**2 + y**2 + z**2))
    yaw = np.arctan2(2.0 * (w*z + x*y), w**2 - x**2 - y**2 + z**2)
    return np.array([roll, pitch, yaw])
#-----------------------------------------------------------------------------#
# Callback function -> CHECK!
def callback_control(msg):
    # Get data from control_algorithmm node
    global control_mode, lever_mode 
    control_mode = msg.pose.covariance[4]
    lever_mode = msg.pose.covariance[18]

# Callback function
def callback_dbwsub(msg):
    # Get data adc from PLC (ADC) --> Feedback (sensor)
    global volt_f_steer, volt_f_throttle, rad_steer, deg_steer, temp_msg
    autonomous_switch = msg.pose.covariance[16]
    cal_steer = msg.pose.covariance[1]      # Steer       
    cal_throttle = msg.pose.covariance[0]   # Throttle? to be confirmed
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
        deg_steer = deg_steer + 1
    elif control_mode == 4: 
        deg_steer += 2
    # if lever_mode == 2:
    #     deg_steer = deg_steer - 3.8
    # if autonomous_switch: 
    #     deg_steer = deg_steer - 9.3
    rad_steer = np.radians(deg_steer) #in radians
    volt_f_throttle = ((cal_throttle / 65535)* 5) #out: (0-5)V value
    temp_msg['v_est'] = msg.pose.covariance[27]
    # print("here")

def gnssFrontCallback(msg):
    # Get position data from the front GNSS
    global temp_msg
    global sensor_data
    sensor_data['x_front'] = msg.pose.pose.position.x
    sensor_data['y_front'] = msg.pose.pose.position.y
    # simple filter based on position changes
    # if position_err > 4: 
    #     # DO:first data initialization
    #     data_bef['xf_bef'] = sensor_data['x_front']
    #     data_bef['yf_bef'] = sensor_data['y_front']
    position_err = np.sqrt((data_bef['xf_bef']-sensor_data['x_front'])**2 + (data_bef['yf_bef']-sensor_data['y_front'])**2)
    if position_err > 0.4 and position_err < 4: 
        # DO:use last data
        sensor_data['x_front'] = data_bef['xf_bef']
        sensor_data['y_front'] = data_bef['yf_bef']
    else: 
        # DO: data is valid, use this data as the position data
        # then, update the data_bef field
        data_bef['xf_bef'] = sensor_data['x_front']
        data_bef['yf_bef'] = sensor_data['y_front']

    # calculate head yaw
    # delta_t = 0.2 #ms (5 Hz)
    # temp_msg['v_est'] = np.sqrt((temp_msg['x_est']-sensor_data['x_front'])**2 + (temp_msg['y_est']-sensor_data['y_front'])**2) / delta_t
    # temp_msg['x_est'] = sensor_data['x_front']
    # temp_msg['y_est'] = sensor_data['y_front']
    temp_msg['yaw_head'] = np.radians(3) + np.arctan2(sensor_data['y_front']-sensor_data['y_rear'],
                                                    sensor_data['x_front']-sensor_data['x_rear'])
    # calculate trailer yaw
    temp_msg['yaw_trailer'] = temp_msg['yaw_head'] - rad_steer
    if np.degrees(rad_steer) < -20 and np.degrees(rad_steer) > -40:
        temp_msg['yaw_trailer'] += 0.03
    elif np.degrees(rad_steer) < -40:
        temp_msg['yaw_trailer'] += 0.065
    elif np.degrees(rad_steer) > 20 and np.degrees(rad_steer) < 40:
        temp_msg['yaw_trailer'] -= 0.007
    elif np.degrees(rad_steer) > 40:
        temp_msg['yaw_trailer'] -= 0.02

def gnssRearCallback(msg):
    # Get position data from the front GNSS
    global temp_msg
    global sensor_data
    sensor_data['x_rear'] = msg.pose.pose.position.x
    sensor_data['y_rear'] = msg.pose.pose.position.y
    # simple filter based on position changes
    # if position_err > 4: 
    #     # DO:first data initialization
    #     data_bef['xf_bef'] = sensor_data['x_front']
    #     data_bef['yf_bef'] = sensor_data['y_front']
    position_err = np.sqrt((data_bef['xr_bef']-sensor_data['x_rear'])**2 + (data_bef['yr_bef']-sensor_data['y_rear'])**2)
    if position_err > 0.4 and position_err < 4: 
        # DO:use last data
        sensor_data['x_rear'] = data_bef['xr_bef']
        sensor_data['y_rear'] = data_bef['yr_bef']
    else: 
        # DO: data is valid, use this data as the position data
        # then, update the data_bef field
        data_bef['xr_bef'] = sensor_data['x_rear']
        data_bef['yr_bef'] = sensor_data['y_rear']

    # from HAM notes
    # coordinates transformation
    X_rhc = sensor_data['x_rear'] + 0.75 * np.cos(temp_msg['yaw_head']-(np.pi/2))
    Y_rhc = sensor_data['y_rear'] + 0.75 * np.sin(temp_msg['yaw_head']-(np.pi/2))
    X_cfw = X_rhc + 1.5 * np.cos(temp_msg['yaw_head']-(np.pi))
    Y_cfw = Y_rhc + 1.5 * np.sin(temp_msg['yaw_head']-(np.pi))
    X_crw = X_cfw + 11.291 * np.cos(temp_msg['yaw_trailer'] - np.pi)
    Y_crw = Y_cfw + 11.291 * np.sin(temp_msg['yaw_trailer'] - np.pi)

    sensor_data['x_crw'] = X_crw
    sensor_data['y_crw'] = Y_crw
    
    delta_t = 0.2 #ms (5 Hz)
    # temp_msg['v_est'] = np.sqrt((temp_msg['x_est']-sensor_data['x_crw'])**2 + (temp_msg['y_est']-sensor_data['y_crw'])**2) / delta_t
    temp_msg['x_est'] = X_crw
    temp_msg['y_est'] = Y_crw

    # calculate head yaw
    temp_msg['yaw_head'] = np.radians(3) + np.arctan2(sensor_data['y_front']-sensor_data['y_rear'],
                                                    sensor_data['x_front']-sensor_data['x_rear'])
    # calculate trailer yaw
    temp_msg['yaw_trailer'] = temp_msg['yaw_head'] - rad_steer
    if np.degrees(rad_steer) < -20 and np.degrees(rad_steer) > -40:
        temp_msg['yaw_trailer'] += 0.03
    elif np.degrees(rad_steer) < -40:
        temp_msg['yaw_trailer'] += 0.065
    elif np.degrees(rad_steer) > 20 and np.degrees(rad_steer) < 40:
        temp_msg['yaw_trailer'] -= 0.007
    elif np.degrees(rad_steer) > 40:
        temp_msg['yaw_trailer'] -= 0.02

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
                
#-----------------------------------------------------------------------------#
# subscriber initialization
rospy.Subscriber('/dbw_pubsub', PoseWithCovarianceStamped, callback_dbwsub)
rospy.Subscriber('/utm', Odometry, gnssFrontCallback)
rospy.Subscriber('/utm2', Odometry, gnssRearCallback)
rospy.Subscriber('/imu/data', Imu, callback_imu)
rospy.Subscriber('/control_algorithm', PoseWithCovarianceStamped, callback_control)
#-----------------------------------------------------------------------------#
while not rospy.is_shutdown():
    # list the message to publish
    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.pose.covariance[0] = volt_f_steer #steering angle in volt
    pub_msg.pose.covariance[1] = deg_steer #steering angle in degree
    pub_msg.pose.covariance[2] = rad_steer #steering angle in radian
    pub_msg.pose.covariance[3] = volt_f_throttle #throttle input voltage
    pub_msg.pose.covariance[4] = temp_msg['x_est'] #sensor_data['x_rear'] #temp_msg['x_est']
    pub_msg.pose.covariance[5] = temp_msg['y_est'] #sensor_data['y_rear'] 
    pub_msg.pose.covariance[6] = temp_msg['v_est']
    pub_msg.pose.covariance[7] = temp_msg['yaw_trailer']
    pub_msg.pose.covariance[8] = temp_msg['yaw_head']
    pub_msg.pose.covariance[9] = np.degrees(imu_roll)
    pub_msg.pose.covariance[10] = np.degrees(imu_pitch)
    pub_msg.pose.covariance[11] = np.degrees(imu_yaw)

    # Publish the message
    pub.publish(pub_msg)

    ### Wait until the next loop
    rate.sleep()
