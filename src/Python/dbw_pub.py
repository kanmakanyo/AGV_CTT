#! /usr/bin/env python3
# CINOVASI - HAM MAF
# Node Name: DBW_Sub
# Node No : 2
# This node is used to publish the data/command from the control algorithm to the PLC

#-----------------------------------------------------------------------------#
# function/library declaration
import rospy
import socket
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
#-----------------------------------------------------------------------------#
# create header for publisher node
# Message publisher initialization
rospy.init_node('dbw_publisher')
freq = 20 # Hz
pub = rospy.Publisher('/dbw_pub', PoseWithCovarianceStamped, queue_size=1)
rate = rospy.Rate(freq) # Hz
pub_msg = PoseWithCovarianceStamped()
pub_msg.header.frame_id = 'dbw_publisher_node'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()
#-----------------------------------------------------------------------------#
# Initialization parameter
speed_sp = 0
steer_sp = 0
RECEIVED_CONTROL_SIGNAL = False
RECEIVED_STATE_VAL = False
reference_voltage = rospy.get_param('~Reference_Voltage', 24.62) #check!
#-----------------------------------------------------------------------------#
# Callback function
def callback_control(msg):
    # Get data from control_algorithmm node
    global speed_sp, steer_sp, RECEIVED_CONTROL_SIGNAL
    speed_sp = msg.pose.covariance[1] #in m/s
    steer_sp = msg.pose.covariance[0] #in rad
    RECEIVED_CONTROL_SIGNAL = True

def callback_Data_Conv(msg): 
    # used to receive actual speed and steering angle data
    global RECEIVED_STATE_VAL, steer_now, v
    steer_now = msg.pose.covariance[2] #in radian
    v = msg.pose.covariance[7] #v now
    RECEIVED_STATE_VAL = True
#-----------------------------------------------------------------------------#
# PLC TCP/IP part
ip = rospy.get_param('~ip', "192.168.1.2")
port = rospy.get_param('~port', 4545)

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.connect((ip, port))
#-----------------------------------------------------------------------------#
# subscriber initialization
rospy.Subscriber('/control_algorithm', PoseWithCovarianceStamped, callback_control)
rospy.Subscriber('/data_conv', PoseWithCovarianceStamped, callback_Data_Conv)
#-----------------------------------------------------------------------------#
while not rospy.is_shutdown():
    if RECEIVED_CONTROL_SIGNAL and RECEIVED_STATE_VAL: #send only when state and control signal received
        # First, calculate the value that needed to be sent to the PLC
        if steer_now<(steer_sp+0.0174) and steer_now>(steer_sp-0.0174):
            #Do: keep the steering angle position
            steer_command = 2.6 #volt, send to step up at PLC
        if steer_now<steer_sp: 
            #Do: turn left, with some value
            steer_command = 2.2 #volt, send to step up at PLC
        elif steer_now>steer_sp: 
            #Do: turn right, with some value
            steer_command = 3 #volt, send to step up at PLC

        # Then, calculate the propulsion percentage needed to be sent to the PLC
        if v >= speed_sp: 
            #Do: give zero traction 
            speed_command = 0.2 #volt
        elif v < speed_sp: 
            #Do: give some propulsion value
            speed_command = 0.9 #volt
    else: 
        #Keep the vehicle steady!
        steer_command = 2.6
        speed_command = 0.2
    
    # Conversi data from input (volt) to ADC PLC
    # Steer
    volt2ADC_steer = round((steer_command/reference_voltage)*65535)  
    split_steer = list(str(volt2ADC_steer))
    if len(split_steer) == 5:
        cal_steer2 = split_steer[0] + split_steer[1] + split_steer[2] + split_steer[3] + split_steer[4]
    elif len(split_steer) == 4:
        cal_steer2 = str("0") + split_steer[0] + split_steer[1] + split_steer[2] + split_steer[3]
    elif len(split_steer) == 3:
        cal_steer2 = str("00") + split_steer[0] + split_steer[1] + split_steer[2]    
    elif len(split_steer) == 2:
        cal_steer2 = str("000") + split_steer[0] + split_steer[1]
    elif len(split_steer) == 1:
        cal_steer2 = str("0000") + split_steer[0]
    elif len(split_steer) == 0:
        cal_steer2 = str("00000")
    # Throttle
    volt2ADC_throttle = round((speed_command/reference_voltage)*65535)    # Throttle (ADC)
    split_throttle = list(str(volt2ADC_throttle))
    if len(split_throttle) == 5:
        cal_throttle2 = split_throttle[0] + split_throttle[1] + split_throttle[2] + split_throttle[3] + split_throttle[4]
    elif len(split_throttle) == 4:
        cal_throttle2 = str("0") + split_throttle[0] + split_throttle[1] + split_throttle[2] + split_throttle[3]
    elif len(split_throttle) == 3:
        cal_throttle2 = str("00") + split_throttle[0] + split_throttle[1] + split_throttle[2]    
    elif len(split_throttle) == 2:
        cal_throttle2 = str("000") + split_throttle[0] + split_throttle[1]
    elif len(split_throttle) == 1:
        cal_throttle2 = str("0000") + split_throttle[0]
    elif len(split_throttle) == 0:
        cal_throttle2 = str("00000")
    # Sisa
    nol = str("00000000000000000000000000000000000000000000000000000")

    print("send value (steer,prop): "+str(steer_command)+", "+str(speed_command))
    # Send to the PLC
    sendplc = str("@") + cal_steer2 + cal_throttle2 + nol      # format pengiriman data ke PLC "@" Data digital 2-bit
    sendplc = sendplc.encode()
    server.send(sendplc)

    # list the message to publish, for logging data purpose
    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.pose.covariance[0] = steer_command #steering command to PLC
    pub_msg.pose.covariance[1] = speed_command #speed command to PLC

    # Publish the message
    pub.publish(pub_msg)
    ### Wait until the next loop
    rate.sleep()