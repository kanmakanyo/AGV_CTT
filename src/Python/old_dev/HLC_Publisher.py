#!/usr/bin/env python3

#for used in master PC (with ROS)
# using MQTT
import paho.mqtt.client as mqtt
import numpy as np
import time
import rospy
from agv_ctt.msg import data_collector

# MQTT Stuff
broker ="broker.emqx.io"
port = 1883

def connect_mqtt() -> mqtt:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt.Client()
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

# init Data to send: 
cs_lat = 0.785 #steering angle in rad
cs_long = 1
cs_brake = 2000
cs_lamp = 1
steer_now = 45.01
X_now = 558091.9866405
Y_now = 9157952.32
V_now = 0.578
yaw_now = 0.745
yaw_head = 0.785
ultra1 = 102
ultra2 = 104
ultra3 = 102
ultra4 = 104
ultra5 = 102
ultra6 = 104
ultra7 = 102
ultra8 = 104
angle_danger = 15
range_danger = 2.5
lidar_danger_flag = 1
ZC3_to_ECU = 1556
ZC2 = 1986
PT3 = 2000
ZC7 = 2005
ZC8 = 2998
ZC6 = 1756
ECVLAV = 6578
ECVLAR = 8658
PS4 = 1233
LS2 = 1
PS1 = 0
PDS1 = 1
PDS2 = 1
PS2 = 1
TS1 = 0
LS1 = 1
autonomous_switch = 1
emgs_button = 0
YS31_to_PVG = 1
YS32_to_PVG = 1
ESV1415_to_PVG = 1
ECV2J19_to_PVG = 1
B01 = 1
C01 = 1
LS4 = 1
ESVDLAR_ENA = 1
ECVLAR_ENA = 0
ST1 = 1000
idc_lamp = 0

def callback_data_collector(msg):
    global cs_lat, cs_long, cs_brake, cs_lamp
    global steer_now, X_now, Y_now, V_now, yaw_now, yaw_head
    global ultra1, ultra2, ultra3, ultra4, ultra5, ultra6, ultra7, ultra8
    global ZC3_to_ECU, ZC2, PT3, ZC7, ZC8, ZC6, ECVLAR
    global ECVLAV, PS4, LS2, PS1, PDS1, PDS2, YS31_to_PVG
    global YS32_to_PVG, ESV1415_to_PVG, PS2, TS1, LS1 
    global autonomous_switch, emgs_button, ECV2J18_to_PVG
    global B01, C01, LS4, ESVDLAR_ENA, ECVLAR_ENA, ST1
    global angle_danger, range_danger
    global lidar_danger_flag, idc_lamp
    cs_lat = msg.covariance[0]
    cs_long = msg.covariance[1]
    cs_brake = msg.covariance[2]
    cs_lamp = msg.covariance[3]
    steer_now = msg.covariance[4]
    X_now = msg.covariance[5]
    Y_now = msg.covariance[6]
    V_now = msg.covariance[7]
    yaw_now = msg.covariance[8]
    yaw_head = msg.covariance[9]
    ultra1 = msg.covariance[11]
    ultra2 = msg.covariance[12]
    ultra3 = msg.covariance[13]
    ultra4 = msg.covariance[14]
    ultra5 = msg.covariance[15]
    ultra6 = msg.covariance[16]
    ultra7 = msg.covariance[17]
    ultra8 = msg.covariance[18]
    angle_danger = msg.covariance[19]
    range_danger = msg.covariance[20]
    lidar_danger_flag = msg.covariance[21]
    ZC3_to_ECU = msg.covariance[22]
    ZC2 = msg.covariance[23]
    PT3 = msg.covariance[24]
    ZC7 = msg.covariance[25]
    ZC8 = msg.covariance[26]
    ZC6 = msg.covariance[27]
    ECVLAV = msg.covariance[28]
    ECVLAR = msg.covariance[29]
    PS4 = msg.covariance[30]
    LS2 = msg.covariance[31]
    PS1 = msg.covariance[32]
    PDS1 = msg.covariance[33]
    PDS2 = msg.covariance[34]
    PS2 = msg.covariance[35]
    TS1 = msg.covariance[36]
    LS1 = msg.covariance[37]
    autonomous_switch = msg.covariance[38]
    emgs_button = msg.covariance[39]
    YS31_to_PVG = msg.covariance[40]
    YS32_to_PVG = msg.covariance[41]
    ESV1415_to_PVG = msg.covariance[42]
    ECV2J19_to_PVG = msg.covariance[43]
    B01 = msg.covariance[44]
    C01 = msg.covariance[45]
    LS4 = msg.covariance[46]
    ESVDLAR_ENA = msg.covariance[47]
    ECVLAR_ENA = msg.covariance[48]
    ST1 = msg.covariance[49]
    idc_lamp = msg.covariance[10]

# def callback_sub(client, userdata, message):
#     global speed

#     speed = str(message.payload.decode("utf-8"))
#     print(speed)

# def speed_other_callback(client, userdata, message):
#     global speed_other

#     speed_other = float(message.payload.decode("utf-8"))
#     cs_long_oth = int(2729.123 * speed_other - 0.014995)
#     if speed_other==0:
#         cs_long_oth = 0
#     client.publish("/EV3_command/speed_actuate_other",cs_long_oth,qos=0)
rospy.init_node('mqtt_pub')
rospy.Subscriber('/data_hmi', data_collector, callback_data_collector)
client = connect_mqtt()
client.loop_start()

while True:
    # data = 1209012932
    data = "@"+ str(cs_lat) + str(";") + str(cs_long) + str(";") + str(cs_brake) + str(";") + str(cs_lamp) +\
         str(";") + str(steer_now) + str(";") + str(X_now) + str(";") + str(Y_now) + str(";") + str(V_now) +\
         str(";") + str(yaw_now) + str(";") + str(yaw_head) + str(";") + str(ultra1) + str(";") + str(ultra2) +\
         str(";") + str(ultra3) + str(";") + str(ultra4) + str(";") + str(ultra5) + str(";") + str(ultra6) +\
         str(";") + str(ultra7) + str(";") + str(ultra8) + str(";") + str(angle_danger) + str(";") + str(range_danger) +\
         str(";") + str(lidar_danger_flag) + str(";") + str(ZC3_to_ECU) + str(";") + str(ZC2) + str(";") +\
         str(PT3) + str(";") + str(ZC7) + str(";") + str(ZC8) + str(";") + str(ZC6) + str(";") + str(ECVLAV) +\
         str(";") + str(ECVLAR) + str(";") + str(PS4) + str(";") + str(LS2) + str(";") + str(PS1) + str(";") +\
         str(PDS1) + str(";") + str(PDS2) + str(";") + str(PS2) + str(";") + str(TS1) + str(";") + str(LS1) +\
         str(";") + str(autonomous_switch) + str(";") + str(emgs_button) + str(";") + str(YS31_to_PVG) + str(";") +\
         str(YS32_to_PVG) + str(";") + str(ESV1415_to_PVG) + str(";") + str (ECV2J19_to_PVG) + str(";") + str(B01) +\
         str(";") + str(C01) + str(";") + str(LS4) + str(";") + str(ESVDLAR_ENA) + str(";") + str(ECVLAR_ENA) +\
         str(";") + str(ST1) + str(";") + str(idc_lamp)
    client.publish("/AGV_CTT/Data",data,qos=0)
    print(cs_long)

    time.sleep(0.2)