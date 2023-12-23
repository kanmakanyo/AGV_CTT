#! /usr/bin/env python3

import rospy
import socket
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

# generate random integer values
from random import randint

# def pub_data():
#     buffer=randint(0,100)
#     time.sleep(3)      
#     return buffer

def callback(msg):
    # Send data to PLC
    buffersend = msg.linear.x + float(0.7)                      # Steer (volt)
    buffersend2 = msg.linear.y                                  # Throttle (volt)
    buffersend3 = msg.linear.z + float(0.7)                     # Steer inject v2 (volt)

    # print(buffersend3, buffersend2, buffersend)
    # Conversi data from HMI (volt to ADC)
    #volt2ADC_steer = round((buffersend/21.11)*65535)      # Steer (ADC)
    volt2ADC_steer = round((buffersend/24.62)*65535)    # Steer v2 (ADC)  
    volt2ADC_throttle = round((buffersend2/24.62)*65535)    # Throttle (ADC)
    nol = str("00000000000000000000000000000000000000000000000000000")
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
    
    #print(cal_steer2)

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


    # Send to PLC
    sendplc = str("@") + cal_steer2 + cal_throttle2 + nol      # format pengiriman data ke PLC "@" Data digital 2-bit
    sendplc = sendplc.encode()
    server.send(sendplc)                
    #print('Send: ' + str(sendplc))         

if __name__ == "__main__":
    ip = "192.168.1.2"
    port = 4545

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.connect((ip, port))
    rospy.init_node('PLC_data')
    sub = rospy.Subscriber('/pyqt_topic', Twist, callback)
    pub_plc = Twist()
    pub = rospy.Publisher('/PLC_data', Twist, queue_size=12)
    while not rospy.is_shutdown():
        # buffer_gen=pub_data()  # debug random number

        # Recive data from PLC
        buffer = server.recv(65536)
        buffer = buffer.decode("utf-8")
        a=buffer.split("@")
        lis = list(a[1])
        cal_steer = int(lis[0])*10000 + int(lis[1])*1000 + int(lis[2])*100 + int(lis[3])*10 + int(lis[4])
        cal_throttle = int(lis[5])*10000 + int(lis[6])*1000 + int(lis[7])*100 + int(lis[8])*10 + int(lis[9])

        # Conversi data from PLC (ADC) to volt
        volt_steer = ((cal_steer / 65535)* 24.62) - 0.6
        
        # (Steer) linearisasi 24v to 5v 
        # a_steer = float(5.07957792207792)
        # c_steer = float(0.075367965367968)
        # steer24v_5v = a_steer * float(volt_steer) + c_steer

        volt_throttle = (cal_throttle/65535)*5      
        #print(f"Steering: {cal_steer}")
        #print(f"Steering volt: {volt_steer}")
        #print(f"Throttle: {cal_throttle}")        
        pub_plc.angular.x = volt_steer       # Publish Steering
        pub_plc.angular.y = volt_throttle    # Publish Throttle    
        pub.publish(pub_plc)
      
rospy.spin()
# -------------------------------------------- end of code ---------------------------------

