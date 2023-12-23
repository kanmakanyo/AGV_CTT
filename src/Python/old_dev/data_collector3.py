#! /usr/bin/env python3
# CINOVASI - HAM MAF
# Node name : data_collector
# Node number : 
# This node is used to collect every data in ROS node

#-----------------------------------------------------------------------------#
# Function/library declaration
from operator import index
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from agv_ctt.msg import data_collector
from sensor_msgs.msg import LaserScan

rospy.init_node('data_hmi')
freq = 20 # Hz
# pub = rospy.Publisher('/data_hmi', PoseWithCovarianceStamped, queue_size=11)
pub = rospy.Publisher('/data_hmi', data_collector, queue_size=11)
rate = rospy.Rate(freq) # Hz
pub_msg = data_collector()
pub_msg.header.frame_id = 'data_hmi_node'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()
#-----------------------------------------------------------------------------#
angle_danger = 0
range_danger = 10
lidar_danger_flag = 0
cs_lat, cs_long, cs_brake, cs_lamp = 0, 0, 0, 0
steer_now, X_now, Y_now, V_now, yaw_now, yaw_head = 0, 0, 0, 0, 0, 0
idc_lamp = 0
ultra1, ultra2, ultra3, ultra4, ultra5, ultra6, ultra7, ultra8 = 0, 0, 0, 0, 0, 0, 0, 0
ZC3_to_ECU, ZC2, PT3, ZC7, ZC8, ZC6, ECVLAR = 0, 0, 0, 0, 0, 0, 0
ECVLAV, PS4, LS2, PS1, PDS1, PDS2, YS31_to_PVG = 0, 0, 0, 0, 0, 0, 0
YS32_to_PVG, ESV1415_to_PVG, PS2, TS1, LS1 = 0, 0, 0, 0, 0
autonomous_switch, emgs_button, ECV2J18_to_PVG = 0, 0, 0
B01, C01, LS4, ESVDLAR_ENA, ECVLAR_ENA, ST1 = 0, 0, 0, 0, 0, 0
#-----------------------------------------------------------------------------#
# Callback Function
def callback_controlAlgorithm(msg):
    global cs_lat, cs_long, cs_brake, cs_lamp
    cs_lat = msg.pose.covariance[0]    #steering angle in rad
    cs_long = msg.pose.covariance[1]   #speed set point in m/s
    cs_brake = msg.pose.covariance[2]  #braking percentage
    cs_lamp = msg.pose.covariance[3]   #autonomous lamp

def callback_dataConv(msg):
    global steer_now, X_now, Y_now, V_now, yaw_now, yaw_head
    steer_now = msg.pose.covariance[1] #degree
    X_now = msg.pose.covariance[4]
    Y_now = msg.pose.covariance[5]
    V_now = msg.pose.covariance[6]
    yaw_now = msg.pose.covariance[7]
    yaw_head = msg.pose.covariance[8]

# def callback_idcLamp(msg):
#     global idc_lamp
#     idc_lamp = msg.pose.covariance[0]
#     print(idc_lamp)

def callback_utrasonik(msg):
    global ultra1, ultra2, ultra3, ultra4, ultra5, ultra6, ultra7, ultra8
    ultra1 = msg.pose.covariance[0]
    ultra2 = msg.pose.covariance[1]
    ultra3 = msg.pose.covariance[2]
    ultra4 = msg.pose.covariance[3]
    ultra5 = msg.pose.covariance[4]
    ultra6 = msg.pose.covariance[5]
    ultra7 = msg.pose.covariance[6]
    ultra8 = msg.pose.covariance[7]

def callback_dbw_pubsub(msg):
    global ZC3_to_ECU, ZC2, PT3, ZC7, ZC8, ZC6, ECVLAR
    global ECVLAV, PS4, LS2, PS1, PDS1, PDS2, YS31_to_PVG
    global YS32_to_PVG, ESV1415_to_PVG, PS2, TS1, LS1 
    global autonomous_switch, emgs_button, ECV2J18_to_PVG
    global B01, C01, LS4, ESVDLAR_ENA, ECVLAR_ENA, ST1  
    
    ZC3_to_ECU = msg.pose.covariance[0] 
    ZC2 = msg.pose.covariance[1]
    PT3 = msg.pose.covariance[2]
    ZC7 = msg.pose.covariance[3]
    ZC8 = msg.pose.covariance[4]
    ZC6 = msg.pose.covariance[5]
    ECVLAV = msg.pose.covariance[6]
    ECVLAR = msg.pose.covariance[7]
    PS4 = msg.pose.covariance[8]
    LS2 = msg.pose.covariance[9]
    PS1 = msg.pose.covariance[10]
    PDS1 = msg.pose.covariance[11]
    PDS2 = msg.pose.covariance[12]
    PS2 = msg.pose.covariance[13]
    TS1 = msg.pose.covariance[14]
    LS1 = msg.pose.covariance[15]
    autonomous_switch = msg.pose.covariance[16]
    emgs_button = msg.pose.covariance[17]
    YS31_to_PVG = msg.pose.covariance[18]
    YS32_to_PVG = msg.pose.covariance[19]
    ESV1415_to_PVG = msg.pose.covariance[20]
    ECV2J19_to_PVG = msg.pose.covariance[21]
    B01 = msg.pose.covariance[22]
    C01 = msg.pose.covariance[23]
    LS4 = msg.pose.covariance[24]
    ESVDLAR_ENA = msg.pose.covariance[25]
    ECVLAR_ENA = msg.pose.covariance[26]

def callback_lidar(msg):
    global angle_danger, range_danger
    global lidar_danger_flag
    lidar_angle = msg.intensities
    lidar_range = msg.ranges
    angle_filtered=[]
    range_filtered=[]
    range_danger = 10
    for i in range(len(lidar_angle)):
        if lidar_range[i] < 4: #lower than 4 meters
            range_filtered.append(lidar_range[i])
            angle_filtered.append(lidar_angle[i])
    
    if len(range_filtered)>0:
        index_min = np.argmin(range_filtered)
        angle_danger = angle_filtered[index_min]
        range_danger = range_filtered[index_min]

    if range_danger < 3.5: 
        lidar_danger_flag = 1
    else: 
        lidar_danger_flag = 0

# Subscriber Initialization
rospy.Subscriber('/control_algorithm', PoseWithCovarianceStamped, callback_controlAlgorithm)
rospy.Subscriber('/data_conv', PoseWithCovarianceStamped, callback_dataConv)
# rospy.Subscriber('/idc_lamp', PoseWithCovarianceStamped, callback_idcLamp)
rospy.Subscriber('/ultrasonic', PoseWithCovarianceStamped, callback_utrasonik)
rospy.Subscriber('/dbw_pubsub', PoseWithCovarianceStamped, callback_dbw_pubsub)
rospy.Subscriber('/lidar', LaserScan, callback_lidar) #TBD

# List msg to publish
while not rospy.is_shutdown():
    # list the message to publish
    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.covariance[0] = cs_lat #steering angle in rad
    pub_msg.covariance[1] = cs_long #speed set point in m/s
    pub_msg.covariance[2] = cs_brake #braking percentage
    pub_msg.covariance[3] = cs_lamp #command indicator autonomous
    pub_msg.covariance[4] = steer_now #steer in degree
    pub_msg.covariance[5] = X_now #sensor_data['y_rear'] 
    pub_msg.covariance[6] = Y_now
    pub_msg.covariance[7] = V_now
    pub_msg.covariance[8] = yaw_now
    pub_msg.covariance[9] = yaw_head
    # pub_msg.covariance[10]= idc_lamp # indikator lampu otonom
    pub_msg.covariance[11]= ultra1
    pub_msg.covariance[12]= ultra2
    pub_msg.covariance[13]= ultra3
    pub_msg.covariance[14]= ultra4
    pub_msg.covariance[15]= ultra5
    pub_msg.covariance[16]= ultra6
    pub_msg.covariance[17]= ultra7
    pub_msg.covariance[18]= ultra8
    pub_msg.covariance[19]= angle_danger #TBD
    pub_msg.covariance[20]= range_danger
    pub_msg.covariance[21]= lidar_danger_flag
    pub_msg.covariance[22] = ZC3_to_ECU
    pub_msg.covariance[23] = ZC2
    pub_msg.covariance[24] = PT3
    pub_msg.covariance[25] = ZC7
    pub_msg.covariance[26] = ZC8
    pub_msg.covariance[27] = ZC6
    pub_msg.covariance[28] = ECVLAV
    pub_msg.covariance[29] = ECVLAR
    pub_msg.covariance[30] = PS4
    pub_msg.covariance[31] = LS2
    pub_msg.covariance[32] = PS1
    pub_msg.covariance[33] = PDS1
    pub_msg.covariance[34] = PDS2
    pub_msg.covariance[35] = PS2
    pub_msg.covariance[36] = TS1
    pub_msg.covariance[37] = LS1
    pub_msg.covariance[38] = autonomous_switch
    pub_msg.covariance[39] = emgs_button
    pub_msg.covariance[40] = YS31_to_PVG
    pub_msg.covariance[41] = YS32_to_PVG
    pub_msg.covariance[42] = ESV1415_to_PVG
    pub_msg.covariance[43] = ECV2J18_to_PVG
    pub_msg.covariance[44] = B01
    pub_msg.covariance[45] = C01
    pub_msg.covariance[46] = LS4
    pub_msg.covariance[47] = ESVDLAR_ENA
    pub_msg.covariance[48] = ECVLAR_ENA
    pub_msg.covariance[49] = ST1
    
    # Publish the message
    pub.publish(pub_msg)

    ### Wait until the next loop
    rate.sleep()
