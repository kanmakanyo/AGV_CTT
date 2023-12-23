import socket
import numpy as np
import time

ip = "localhost"
port = 2000

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.connect((ip, port))

while(True):
    # buffer = server.recv(65536)
    # buffer = buffer.decode("utf-8")
    # a=buffer.split("@")
    # lis = list(a[1])

    # # data from PLC
    # ZC5 = 
    # SQ1 = 
    # SQ2 = 

    # data to PLC 
    # dummy variable
    ZC2_target = str("00000")
    ESV1415_ENA_target = str("0")
    ECV2J19_ENA_target = str("0")
    speed_target = str("0000")
    YS3_1_SIG_target = str("0")
    YS3_2_SIG_target = str("0")
    ESVDLAR_ENA_SIG_target = str("0")
    ECVLAR_ENA_SIG_target = str("0")
    ZC6_target = str("00000")
    ZC7_target = str("00000")
    B01_ENA_target = str("0")
    C01_ENA_target = str("0")
    auto_lamp_target = str("0")
    brake_target_ = str("00000")
    ESDVDAR_ENA_target = str("0")
    # test variable
    ZC5_target = str("1234")
    ECVDAR_E25_SIG = str("1")
    ECVDAR_E24_SIG = str("1")
    ESVI_target = str("1")
    ESVD_target = str("1")

    sendplc = str("@") + ZC2_target + ESV1415_ENA_target + ECV2J19_ENA_target + speed_target + \
        YS3_1_SIG_target + YS3_2_SIG_target + ESVDLAR_ENA_SIG_target + ECVLAR_ENA_SIG_target + \
        ZC6_target + ZC7_target + B01_ENA_target + C01_ENA_target + auto_lamp_target + brake_target_ 
    # sendplc = sendplc.encode()
    # server.send(sendplc)