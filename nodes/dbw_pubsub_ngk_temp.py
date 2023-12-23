#! /usr/bin/env python3
# CINOVASI - HAM MAF
# Node Name: DBW_Sub Pub
# Node No : 3
# This node is used to subscribe every data from PLC to the High Level Controller using TCPIP
# Also used to publish the control signal to the PLC

#-----------------------------------------------------------------------------#
# Function/Library declaration
import rospy
import socket
import numpy as np
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
#-----------------------------------------------------------------------------#
# create header for publisher node
# Message publisher initialization
rospy.init_node('dbw_subscriber_publisher')
freq = 20 # Hz
pub = rospy.Publisher('/dbw_pubsub', PoseWithCovarianceStamped, queue_size=1)
rate = rospy.Rate(freq) # Hz
pub_msg = PoseWithCovarianceStamped()
pub_msg.header.frame_id = 'dbw_subscriber_node'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()
#-----------------------------------------------------------------------------#
# Initialization parameter
kp_left = 0.47*4
kp_right = 0.29*4
err_left = 0
err_right = 0
steer_command = 2.6 + float(0.6) 
speed_command = 0.2 # 0.2 last value. Now new value edited 18 okt 2023
speed_sp = 0
steer_sp = 0
v = 0
time_speed = time.time()
time_left = time.time()
time_right = time.time()
dt_speed = 0
dt_left = 0
dt_right = 0
RECEIVED_CONTROL_SIGNAL = False
RECEIVED_STATE_VAL = False
reference_voltage = rospy.get_param('~Reference_Voltage', 20) #check!
cs_lamp = 0
YS3_1_SIG = 0
YS3_2_SIG = 0
ESV1415_ENA = 0
ECV2J19_ENA = 0
control_mode = 0
lever_mode = 0
brake_target = 1670
ZC6_SIG = 4610 #down
ZC7_SIG = 4594 #down
ZC6_SIG_HLC = 4610 #down
ZC7_SIG_HLC = 4594 #down
lift_ena = 0
brake_reach = 0
cs_brake = 0
#========New for Rear Steering=========
rear_steer_command = 2.6 + float(0.6) 
ECVDAR_ENA = 0 
ESVI_SIG = 0 
ESVD_SIG = 0 
#======================================
#-----------------------------------------------------------------------------#
# Callback function -> CHECK!
def callback_control(msg):
    # Get data from control_algorithmm node
    global speed_sp, steer_sp, RECEIVED_CONTROL_SIGNAL, cs_lamp, lever_mode
    global control_mode, lift_ena, ZC6_SIG_HLC, ZC7_SIG_HLC, cs_brake
    speed_sp = msg.pose.covariance[1] #in m/s
    steer_sp = msg.pose.covariance[0] #in rad
    control_mode = msg.pose.covariance[4] 
    # brake_sp = msg.pose.covariance[2] #in percentage or pressure value <-- TBD
    cs_lamp = msg.pose.covariance[3] #autonomous lamp state
    lever_mode = msg.pose.covariance[18] #lever mode
    ZC6_SIG_HLC = int(msg.pose.covariance[21])
    ZC7_SIG_HLC = int(msg.pose.covariance[22])
    lift_ena = msg.pose.covariance[20]
    cs_brake = msg.pose.covariance[2]

    RECEIVED_CONTROL_SIGNAL = True

def callback_Data_Conv(msg): 
    # used to receive actual speed and steering angle data
    global RECEIVED_STATE_VAL, steer_now, v, lamp
    lamp = msg.pose.covariance[0]
    # print(lamp)
    steer_now = msg.pose.covariance[2] #in radian
    v = msg.pose.covariance[6] #v now
    RECEIVED_STATE_VAL = True
#-----------------------------------------------------------------------------#
# Conversion function -> CHECK!
def steer_ADC_to_rad(ADC_val): 
    volt_f_steer = ((ADC_val / 65535)* reference_voltage) #in volt (CHECK!)
    deg_steer = (volt_f_steer - 2.6603)/0.013 #in degree
    rad_steer = np.radians(deg_steer) #in radians
    return rad_steer

def steer_rad_to_ADC(rad_val):
    steer_ADC = np.degrees(rad_val)*0.013 + 2.6603 #in volt
    steer_ADC = ((steer_ADC)/reference_voltage)*65535 #in ADC value
    return int(steer_ADC)

def lever_signal(lever_mode): 
    if lever_mode == 0: #neutral
        YS3_1_SIG = 0
        YS3_2_SIG = 0
    elif lever_mode == 1: #forward
        YS3_1_SIG = 0
        YS3_2_SIG = 1
    elif lever_mode == 2: #reverse
        YS3_1_SIG = 1
        YS3_2_SIG = 0
    return YS3_1_SIG, YS3_2_SIG

def steer_ena(steer_state):
    if steer_state == 1:
        ESV1415_ENA = 1
        ECV2J19_ENA = 1
    else: 
        ESV1415_ENA = 0
        ECV2J19_ENA = 0
    return ESV1415_ENA, ECV2J19_ENA

def lift_enabler(lift_ena):
    if lift_ena == 0: #stop
        ESVDLAR_ENA_SIG = 0
        ECVLAR_ENA_SIG = 0 
    elif lift_ena == 1: #go up
        ESVDLAR_ENA_SIG = 0
        ECVLAR_ENA_SIG = 1
    elif lift_ena == 2: #go down
        ESVDLAR_ENA_SIG = 1
        ECVLAR_ENA_SIG = 1
    return ESVDLAR_ENA_SIG, ECVLAR_ENA_SIG
#------------------------------New-----------------------------------------
def rear_steer_ADC_to_rad(ADC_val): 
    volt_f_steer = ((ADC_val / 65535)* reference_voltage) #in volt (CHECK!)     # Perlu kalibrasi
    deg_steer = (volt_f_steer - 2.6603)/0.013 #in degree                        # Perlu kalibrasi
    rad_steer = np.radians(deg_steer) #in radians                               
    return rad_steer

def rear_steer_rad_to_ADC(rad_val):
    steer_ADC = np.degrees(rad_val)*0.013 + 2.6603 #in volt             # Perlu kalibrasi 
    steer_ADC = ((steer_ADC)/reference_voltage)*65535 #in ADC value     # Perlu kalibrasi
    return int(steer_ADC)
#-----------------------------------------------------------------------------#
# subscriber initialization
rospy.Subscriber('/control_algorithm', PoseWithCovarianceStamped, callback_control)
rospy.Subscriber('/data_conv', PoseWithCovarianceStamped, callback_Data_Conv)
rospy.Subscriber('/idc_lamp', PoseWithCovarianceStamped, callback_Data_Conv)
#-----------------------------------------------------------------------------#
# TCP setting
# print("sssss")
ip = rospy.get_param('~ip', "192.168.1.10")
port = rospy.get_param('~port', 4545)
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.connect((ip, port))
#-----------------------------------------------------------------------------#
while not rospy.is_shutdown():
    # Receive data from PLC
    #-----------------------------------------------------------------------------#
    buffer = server.recv(65536)
    buffer = buffer.decode("utf-8")
    a=buffer.split("@")
    lis = list(a[1])
    # Decode reading value2 from PLC
    # 0-4 : ZC3 to ECU
    # 5-9 : ZC2
    # 10-14 : PT3
    # 15-19 : ZC7
    # 20-24 : ZC8
    # 25-29 : ZC6
    # 30-34 : ECVLAV
    # 35-39 : ECVLAR
    # 40    : PS4
    # 41    : LS2
    # 42    : PS1
    # 43    : PDS1
    # 44    : PDS2
    # 45    : PS2
    # 46    : TS1
    # 47    : LS1
    # 48    : Autonomous Switch
    # 49    : Emergency Button
    # 50    : YS31_to_PVG
    # 51    : YS32_to_PVG
    # 52    : ESV1415_to_PVG
    # 53    : ECV2 J19 to PVG
    # 54    : B01 NC
    # 55    : C01 NC
    # 56    : LS4   
    # 57    : ESVDLAR
    # 58    : ECVLAR
    # 59-62 : ST1
    # 63-67 : ZC3 to ECU (PID)
    # 68-72 : ECV2J18 to PVG (PID)
    # 74-78 : ZC5 
    # 79    : ECVDAR_ENA to PVG 
    # 80    : SQ1
    # 81    : SQ2
    ZC3_to_ECU = int(lis[0])*10000 + int(lis[1])*1000 + int(lis[2])*100 + int(lis[3])*10 + int(lis[4])
    ZC2 = int(lis[5])*10000 + int(lis[6])*1000 + int(lis[7])*100 + int(lis[8])*10 + int(lis[9])
    PT3 = int(lis[10])*10000 + int(lis[11])*1000 + int(lis[12])*100 + int(lis[13])*10 + int(lis[14])
    ZC7 = int(lis[15])*10000 + int(lis[16])*1000 + int(lis[17])*100 + int(lis[18])*10 + int(lis[19])
    ZC8 = int(lis[20])*10000 + int(lis[21])*1000 + int(lis[22])*100 + int(lis[23])*10 + int(lis[24])
    ZC6 = int(lis[25])*10000 + int(lis[26])*1000 + int(lis[27])*100 + int(lis[28])*10 + int(lis[29])
    ECVLAV = int(lis[30])*10000 + int(lis[31])*1000 + int(lis[32])*100 + int(lis[33])*10 + int(lis[34])
    ECVLAR = int(lis[35])*10000 + int(lis[36])*1000 + int(lis[37])*100 + int(lis[38])*10 + int(lis[39])
    PS4 = int(lis[40])
    LS2 = int(lis[41])
    PS1 = int(lis[42])
    PDS1 = int(lis[43])
    PDS2 = int(lis[44])
    PS2 = int(lis[45])
    TS1 = int(lis[46])
    LS1 = int(lis[47])
    autonomous_switch = int(lis[48])
    emgs_button = int(lis[49])
    YS31_to_PVG = int(lis[50])
    YS32_to_PVG = int(lis[51])
    ESV1415_to_PVG = int(lis[52])
    ECV2J19_to_PVG = int(lis[53])
    B01 = int(lis[54])
    C01 = int(lis[55])
    LS4 = int(lis[56])
    ESVDLAR_ENA = int(lis[57])
    ECVLAR_ENA = int(lis[58])
    ST1 = int(lis[59])*1000 + int(lis[60])*100 + int(lis[61])*10 + int(lis[62])
    # ST1 = ST1/100 #in km/h
    # ST1 = ST1/3.6 #in m/s
    ZC3_ECU_PID = int(lis[63])*10000 + int(lis[64])*1000 + int(lis[65])*100 + int(lis[66])*10 + int(lis[67])
    ECV2J18_PID = int(lis[68])*10000 + int(lis[69])*1000 + int(lis[70])*100 + int(lis[71])*10 + int(lis[72])
    ECV2J18_PID = int(lis[68])*10000 + int(lis[69])*1000 + int(lis[70])*100 + int(lis[71])*10 + int(lis[72])
    ZC5 = int(lis[73])*10000 + int(lis[74])*1000 + int(lis[75])*100 + int(lis[76])*10 + int(lis[77])
    print(f"ZC5 : {ZC5}")
    ECVDARE24_PID = int(lis[78] )*10000 + int(lis[79])*1000 + int(lis[80])*100 + int(lis[81])*10 + int(lis[82])
    print(f"ECVDARE24_PID : {ECVDARE24_PID}")
    SQ1 = int(lis[83])
    print(f"SQ1 : {SQ1}")
    SQ2 = int(lis[84])
    print(f"SQ2 : {SQ2}") 
    ECVDARE25_to_PVG = int(lis[85])        
    print(f"ECVDARE25_to_PVG [Enabler Rear Steer] : {ECVDARE25_to_PVG}")
    
    PT3 = (PT3/65535)*20 #in volt
    PT3 = int(39.502 * PT3 - 21.016)
    # cal_steer = int(lis[0])*10000 + int(lis[1])*1000 + int(lis[2])*100 + int(lis[3])*10 + int(lis[4])
    # cal_throttle = int(lis[5])*10000 + int(lis[6])*1000 + int(lis[7])*100 + int(lis[8])*10 + int(lis[9])
    #-----------------------------------------------------------------------------#
    # Data send: 
    # steering angle target in form of ZC2 ADC
    # steering enabler signal ESV1415 and ECV2J19
    # speed target in form of m/s or ST1 pulse count? <-- check again
    # YS3(1) and YS3(2) state in form of digital output high/low
    # Lifting enabler state ESVDLAR_ENA and ECVLAR_ENA in form of digital output high/low
    # Lifting height in form of ZC6 and ZC7 ADC voltage
    # Braking command to B01 and C01
    # Voltage 0-5V offset to PLC
    # Vref to PLC
    if RECEIVED_CONTROL_SIGNAL and RECEIVED_STATE_VAL and cs_lamp:# and autonomous_switch: #send only when cs_lamp, state and control signal true
        # Enabling steer
        ESV1415_ENA, ECV2J19_ENA = steer_ena(1)
        # control the lever state
        YS3_1_SIG, YS3_2_SIG = lever_signal(lever_mode)
        # lifting and brake : hold, and send zero value
        speed_command = speed_sp #in m/s
        # calculate steer command in ZC2 ADC value
        steer_command = steer_rad_to_ADC(steer_sp) #- 2 * 43#in ADC
        if control_mode == 2: 
            steer_command = ZC2
            ESV1415_ENA, ECV2J19_ENA = steer_ena(0)
        elif control_mode == 1:
           steer_command = steer_command #- (2 * 43)
        elif control_mode == 4:
           steer_command = steer_command #- (2 * 43)
        elif control_mode == 6 or control_mode == 7 or control_mode == 8:
            ESV1415_ENA, ECV2J19_ENA = steer_ena(0)
            # speed_command = 0

        # ESV1415_ENA, ECV2J19_ENA = steer_ena(1)
        
        steer_command += (1* 43) #(-3* 43) 

        ESVDLAR_ENA_SIG, ECVLAR_ENA_SIG = lift_enabler(lift_ena)
        ZC6_SIG = ZC6_SIG_HLC
        ZC7_SIG = ZC7_SIG_HLC

        if lift_ena == 0: 
            ZC6_SIG = ZC6
            ZC7_SIG = ZC7
        # elif control_mode == 4: 
        #     if speed_command > 0.4:  
        #         speed_command = 2
        # if lever_mode == 0:
        #     speed_command = 0
        # elif lever_mode == 2:
        #     steer_command = steer_command + (3.8)*43
        
        if YS31_to_PVG and not YS32_to_PVG: #reverse
            # deg_steer = deg_steer - 3.8
            steer_command += (4 * 43)
        # if autonomous_switch: 
        #     steer_command = steer_command + 396 + 43
        # Send the autonomous lamp signal 
        # autonomous_lamp = 1

        rear_steer_command = rear_steer_rad_to_ADC(rear_steer_sp)
        ECVDAR_ENA = 1
        # Cek variabel dibawah 

    else: #not in autonomous mode, or any condition is not fulfilled
        # check if someone forget to turn on the autonomous switch
        # if cs_lamp and not autonomous_switch: 
        #     print("autonomous switch is not turned on! please switch it on!")
        # lifting and brake : hold, and send zero value
        ESVDLAR_ENA_SIG = 0
        ECVLAR_ENA_SIG = 0
        ZC6_SIG = ZC6
        ZC7_SIG = ZC7
        B01_SIG = 0
        C01_SIG = 0
        speed_command = 0 #in m/s
        # steer_command = steer_rad_to_ADC(steer_sp) #- 2* 43 #in ADC
        steer_command = ZC2 #- 2* 43 #in ADC
        # steer_command += (-5 * 43)
        brake_target = PT3 #to be check
        # if autonomous_switch:
        #     brake_target = 2000
            # B01_SIG = 1
            # C01_SIG = 1
        # if autonomous_switch: 
        #     if brake_reach: 
        #         B01_SIG = 0
        #         C01_SIG = 0
        #     else: 
        #         brake_target = 2640
        #         B01_SIG = 1
        #         C01_SIG = 1
        #         if PT3>brake_target:             
        #             brake_reach = 1
        # else: 
        #     brake_reach = 0
            # if PT3 < 
            # B01_SIG = 1
            # C01_SIG = 1
        # steer_command = ZC2 #in ADC
        # if autonomous_switch: 
        #     steer_command = steer_command + 396 + 43
        ESV1415_ENA, ECV2J19_ENA = steer_ena(0)
        # change the lever state to neutral
        YS3_1_SIG, YS3_2_SIG = lever_signal(0)
        # speed and steer set point2to align the steering angle, while the autonomous state has been stopped
        steer_now = steer_ADC_to_rad(ZC2)
        # if steer_now<(steer_sp+np.radians(1)) and steer_now>(steer_sp-np.radians(1)):
        #     #Do: keep the steering angle position
        #     # disable the steering enabler
        #     ESV1415_ENA, ECV2J19_ENA = steer_ena(0)
        #     # change the lever state to neutral
        #     YS3_1_SIG, YS3_2_SIG = lever_signal(0)
        # elif steer_now<steer_sp or steer_now<steer_sp:
        #     # Reach the target position, it will be 0
        #     # enable the steering enabler
        #     ESV1415_ENA, ECV2J19_ENA = steer_ena(1)
        #     # change the lever state to forward
        #     YS3_1_SIG, YS3_2_SIG = lever_signal(1)
        # # Send the autonomous lamp signal 
        # autonomous_lamp = 0
        rear_steer_now = rear_steer_ADC_to_rad(ZC5)
        ECVDAR_ENA = 0 
        SQ1 = 0 
        SQ2 = 0 
        #===============NEW=======================
        # Processing logic for rear steering data : 
        # ZC5 = rear steering angle now 

        # ECVDARE24_PID = command steer now 
        # SQ1 = lock command signal
        # SQ2 = unlock command signal
        # ECVDAR_ENA = rear steering enabler
        #=========================================
    
    # braking
    if cs_brake == 0:
        B01_SIG = 0
        C01_SIG = 0
    elif cs_brake == 1:
        B01_SIG = 1
        C01_SIG = 1
    elif cs_brake == 2:
        B01_SIG = 1
        C01_SIG = 0

    ESV1415_ENA = 1

    # print("ini ena : " + str(ESV1415_ENA))
    # short time tweaking
    # steer_command = ZC2

    #print(steer_command, ZC2)
    #-----------------------------------------------------------------------------#
    # Send Data to PLC
    # 0-4       : steering angle target (ZC2) in form of ADC
    # 5         : ESV1415 signal
    # 6         : ECV2 J19 signal
    # 7-10      : speed target (m/s)
    # 11        : YS3_1_sig
    # 12        : YS3_2_sig
    # 13        : ESVDLAR_ENA_SIG
    # 14        : ECVLAR_ENA_SIG
    # 15-19     : ZC6 set point in ADC
    # 20-24     : ZC7 set point in ADC
    # 25        : B01 enabler
    # 26        : C01 enabler
    # 27        : autonomous lamp
    # 28-32     : Brake target (ADC) 
    # 33-38     : Rear Steering Angle Target (ZC5) in form of ADC
    # 39        : Rear Steering Enabler (ECVDAR E24)
    # 40        : Read Steering Lock Target (ESVI)
    # 41        : Rear Steering Unlock Target (ESVD)

    # Conversi data from input (volt) to ADC PLC
    # Steering angle ZC2 target
    ZC2_to_PLC = list(str(int(steer_command)))
    # print(ZC2_to_PLC)
    if len(ZC2_to_PLC) == 5:
        ZC2_target = ZC2_to_PLC[0] + ZC2_to_PLC[1] + ZC2_to_PLC[2] + ZC2_to_PLC[3] + ZC2_to_PLC[4]
    elif len(ZC2_to_PLC) == 4:
        ZC2_target = str("0") + ZC2_to_PLC[0] + ZC2_to_PLC[1] + ZC2_to_PLC[2] + ZC2_to_PLC[3]
        # print("here", ZC2_target)
        # print()
    elif len(ZC2_to_PLC) == 3:
        ZC2_target = str("00") + ZC2_to_PLC[0] + ZC2_to_PLC[1] + ZC2_to_PLC[2]    
    elif len(ZC2_to_PLC) == 2:
        ZC2_target = str("000") + ZC2_to_PLC[0] + ZC2_to_PLC[1]
    elif len(ZC2_to_PLC) == 1:
        ZC2_target = str("0000") + ZC2_to_PLC[0]
    elif len(ZC2_to_PLC) == 0:
        ZC2_target = str("00000")
    
    # ESV1415 Signal
    ESV1415_ENA_target = list(str(ESV1415_ENA))[0]
    # ECV2J19 signal
    ECV2J19_ENA_target = list(str(ECV2J19_ENA))[0]
    # Speed target
    Speed_SP_to_PLC = str(round(float(speed_command),2)) #2 decimal only
    Speed_SP_to_PLC = Speed_SP_to_PLC.split(".") #split the value before and after decimal
    Speed_val_bef_dec = list(Speed_SP_to_PLC[0]) 
    Speed_val_aft_dec = list(Speed_SP_to_PLC[1])
    if len(Speed_val_bef_dec) == 2: 
        speed_target_bef_dec = Speed_val_bef_dec[0] + Speed_val_bef_dec[1]
    elif len(Speed_val_bef_dec) == 1: 
        speed_target_bef_dec = str("0") + Speed_val_bef_dec[0]
    if len(Speed_val_aft_dec) == 2: 
        speed_target_aft_dec = Speed_val_aft_dec[0] + Speed_val_aft_dec[1]
    elif len(Speed_val_aft_dec) == 1: 
        speed_target_aft_dec = str("0") + Speed_val_aft_dec[0]
    speed_target = speed_target_bef_dec + speed_target_aft_dec
    # YS3_1_sig
    YS3_1_SIG_target = list(str(YS3_1_SIG))[0]
    # YS3_2_sig
    YS3_2_SIG_target = list(str(YS3_2_SIG))[0]
    # ESVDLAR_ENA_SIG
    ESVDLAR_ENA_SIG_target = list(str(ESVDLAR_ENA_SIG))[0]
    # ECVLAR_ENA_SIG
    ECVLAR_ENA_SIG_target = list(str(ECVLAR_ENA_SIG))[0]
    # Front lifting height ZC6 target
    ZC6_to_PLC = list(str(ZC6_SIG))
    if len(ZC6_to_PLC) == 5:
        ZC6_target = ZC6_to_PLC[0] + ZC6_to_PLC[1] + ZC6_to_PLC[2] + ZC6_to_PLC[3] + ZC6_to_PLC[4]
    elif len(ZC6_to_PLC) == 4:
        ZC6_target = str("0") + ZC6_to_PLC[0] + ZC6_to_PLC[1] + ZC6_to_PLC[2] + ZC6_to_PLC[3]
    elif len(ZC6_to_PLC) == 3:
        ZC6_target = str("00") + ZC6_to_PLC[0] + ZC6_to_PLC[1] + ZC6_to_PLC[2]    
    elif len(ZC6_to_PLC) == 2:
        ZC6_target = str("000") + ZC6_to_PLC[0] + ZC6_to_PLC[1]
    elif len(ZC6_to_PLC) == 1:
        ZC6_target = str("0000") + ZC6_to_PLC[0]
    elif len(ZC6_to_PLC) == 0:
        ZC6_target = str("00000")
    # Rear lifting height ZC7 target
    ZC7_to_PLC = list(str(ZC7_SIG))
    if len(ZC7_to_PLC) == 5:
        ZC7_target = ZC7_to_PLC[0] + ZC7_to_PLC[1] + ZC7_to_PLC[2] + ZC7_to_PLC[3] + ZC7_to_PLC[4]
    elif len(ZC7_to_PLC) == 4:
        ZC7_target = str("0") + ZC7_to_PLC[0] + ZC7_to_PLC[1] + ZC7_to_PLC[2] + ZC7_to_PLC[3]
    elif len(ZC7_to_PLC) == 3:
        ZC7_target = str("00") + ZC7_to_PLC[0] + ZC7_to_PLC[1] + ZC7_to_PLC[2]    
    elif len(ZC7_to_PLC) == 2:
        ZC7_target = str("000") + ZC7_to_PLC[0] + ZC7_to_PLC[1]
    elif len(ZC7_to_PLC) == 1:
        ZC7_target = str("0000") + ZC7_to_PLC[0]
    elif len(ZC7_to_PLC) == 0:
        ZC7_target = str("00000")
    # B01 enabler
    B01_ENA_target = list(str(B01_SIG))[0]
    # C01 enabler
    C01_ENA_target = list(str(C01_SIG))[0]
    # Autonom lamp
    auto_lamp_target = list(str(autonomous_switch))[0]
    # Rear lifting height ZC7 target
    brake_to_PLC = list(str(brake_target))
    if len(brake_to_PLC) == 5:
        brake_target_ = brake_to_PLC[0] + brake_to_PLC[1] + brake_to_PLC[2] + brake_to_PLC[3] + brake_to_PLC[4]
    elif len(brake_to_PLC) == 4:
        brake_target_ = str("0") + brake_to_PLC[0] + brake_to_PLC[1] + brake_to_PLC[2] + brake_to_PLC[3]
    elif len(brake_to_PLC) == 3:
        brake_target_ = str("00") + brake_to_PLC[0] + brake_to_PLC[1] + brake_to_PLC[2]    
    elif len(brake_to_PLC) == 2:
        brake_target_ = str("000") + brake_to_PLC[0] + brake_to_PLC[1]
    elif len(brake_to_PLC) == 1:
        brake_target_ = str("0000") + brake_to_PLC[0]
    elif len(brake_to_PLC) == 0:
        brake_target_ = str("00000")
    # ZC5 to PLC (Rear Steering Target)
    # ZC5_target = str("12345")
    ZC5_to_PLC = list(str(int(rear_steer_command)))
    # print(ZC2_to_PLC)
    if len(ZC5_to_PLC) == 5:
        ZC5_target = ZC5_to_PLC[0] + ZC5_to_PLC[1] + ZC5_to_PLC[2] + ZC5_to_PLC[3] + ZC5_to_PLC[4]
    elif len(ZC5_to_PLC) == 4:
        ZC5_target = str("0") + ZC5_to_PLC[0] + ZC5_to_PLC[1] + ZC5_to_PLC[2] + ZC5_to_PLC[3]
        # print("here", ZC2_target)
        # print()
    elif len(ZC5_to_PLC) == 3:
        ZC5_target = str("00") + ZC5_to_PLC[0] + ZC5_to_PLC[1] + ZC5_to_PLC[2]    
    elif len(ZC5_to_PLC) == 2:
        ZC5_target = str("000") + ZC5_to_PLC[0] + ZC5_to_PLC[1]
    elif len(ZC5_to_PLC) == 1:
        ZC5_target = str("0000") + ZC5_to_PLC[0]
    elif len(ZC5_to_PLC) == 0:
        ZC5_target = str("00000")
    # ECVDAR_ENA (Rear Steering ENA)
    ECVDAR_ENA_target = list(str(ECVDAR_ENA))[0]
    # ESVI_SIG 
    ESVI_target = list(str(ESVI_SIG))[0]
    # ESVD_SIG
    ESVD_target = list(str(ESVD_SIG))[0]

    # Testing : 
    # ZC5_target = "12345"
    # ECVDAR_ENA_target = "0"
    # ESVI_target = "1"
    # ESVD_target = "1"
    # print(ZC5_target)

    # Send to the PLC --> format pengiriman data ke PLC "@" Data digital 2-bit
    sendplc = str("@") + ZC2_target + ESV1415_ENA_target + ECV2J19_ENA_target + speed_target + \
        YS3_1_SIG_target + YS3_2_SIG_target + ESVDLAR_ENA_SIG_target + ECVLAR_ENA_SIG_target + \
            ZC6_target + ZC7_target + B01_ENA_target + C01_ENA_target + auto_lamp_target + brake_target_+ \
            ZC5_target + ECVDAR_ENA_target +  ESVI_target + ESVD_target
    # sendplc = str(1342567980983930048302830494895509392920388494033829393848393282)
    # tesst = steer_ADC_to_rad(steer_command)
    # print(np.degrees(tesst), steer_command, ZC2_target, ZC2_to_PLC)
    # print("ini ena : " + str(ESV1415_ENA_target))

    # tesstr = steer_ADC_to_rad(ZC2)
    # print(np.degrees(tesstr), ZC2)
    # print(ZC6_target,ZC7_target, ZC6, ZC7, speed_command)
    # print(brake_target, PT3)
    sendplc = sendplc.encode()
    server.send(sendplc)
    # print(ST1)
    # list the message to publish
    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    # pub_msg.pose.covariance[0] = ZC2 #steering voltage in ADC
    # pub_msg.pose.covariance[1] = cal_throttle #throttle voltage in ADC
    # pub_msg.pose.covariance[2] = steer_command #steering command to PLC
    # pub_msg.pose.covariance[3] = speed_command #speed command to PLC
    pub_msg.pose.covariance[0] = ZC3_to_ECU
    pub_msg.pose.covariance[1] = ZC2
    pub_msg.pose.covariance[2] = PT3
    pub_msg.pose.covariance[3] = ZC7
    pub_msg.pose.covariance[4] = ZC8
    pub_msg.pose.covariance[5] = ZC6
    pub_msg.pose.covariance[6] = ECVLAV
    pub_msg.pose.covariance[7] = ECVLAR
    pub_msg.pose.covariance[8] = PS4
    pub_msg.pose.covariance[9] = LS2
    pub_msg.pose.covariance[10] = PS1
    pub_msg.pose.covariance[11] = PDS1
    pub_msg.pose.covariance[12] = PDS2
    pub_msg.pose.covariance[13] = PS2
    pub_msg.pose.covariance[14] = TS1
    pub_msg.pose.covariance[15] = LS1
    pub_msg.pose.covariance[16] = autonomous_switch
    pub_msg.pose.covariance[17] = emgs_button
    pub_msg.pose.covariance[18] = YS31_to_PVG
    pub_msg.pose.covariance[19] = YS32_to_PVG
    pub_msg.pose.covariance[20] = ESV1415_to_PVG
    pub_msg.pose.covariance[21] = ECV2J19_to_PVG
    pub_msg.pose.covariance[22] = B01
    pub_msg.pose.covariance[23] = C01
    pub_msg.pose.covariance[24] = LS4
    pub_msg.pose.covariance[25] = ESVDLAR_ENA
    pub_msg.pose.covariance[26] = ECVLAR_ENA
    pub_msg.pose.covariance[27] = ST1
    pub_msg.pose.covariance[28] = ZC3_ECU_PID
    pub_msg.pose.covariance[29] = ECV2J18_PID
    #========New===============
    # Publish data control roda belakang
    pub_msg.pose.covariance[30] = ECVDARE24_PID
    pub_msg.pose.covariance[31] = ECVDARE25_to_PVG
    # pub_msg.pose.covariance[29] = 
    # pub_msg.pose.covariance[29] = 

    #==========================

    # Publish the message
    pub.publish(pub_msg)

    ### Wait until the next loop
    rate.sleep()