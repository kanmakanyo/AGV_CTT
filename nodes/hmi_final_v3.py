#! /usr/bin/env python3
# CINOVASI - HAM MAF
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main.ui'
#
# Created by: PyQt5 UI code generator 5.10.1

import sys
import os

# from pyrsistent import s
import rospy
import numpy as np
import time

from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtWidgets import QDialog, QApplication, QWidget, QMainWindow
from PyQt5.QtGui import QPixmap
from geometry_msgs.msg import PoseWithCovarianceStamped

from matplotlib.backends.qt_compat import QtWidgets
from matplotlib.backends.backend_qt5agg import (
    FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from agv_ctt.msg import data_collector

from shapely.geometry import Polygon

# # IMPORT FUNCTIONS
# from ui_functions import *

# docking spec (x,y,orientation,length,width)
# left docking
l_dock_ = [-1.55,-0.5, -1.55, 14.2]
# right docking
r_dock_ = [1.55,-0.5, 1.55, 14.2]
# lower docking
b_dock_ = [-1.55,-0.5, 1.55, -0.5]
# print("HIAAAHAHAHAHAHAHAHAHA")

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        # Subscribe ROS
        rospy.init_node('node_hmi')
        self.sub = rospy.Subscriber('/data_hmi', data_collector, self.callback_data_collector)
        
        # Publish ROS
        self.pub = rospy.Publisher('/idc_lamp', PoseWithCovarianceStamped, queue_size=30)
        self.msg = PoseWithCovarianceStamped()

        # Counter to compensate 
        self.counter = 0
        self.countermax = 200

        # inisiasi variabel
        self.cs_lat = 0
        self.cs_long = 0
        self.cs_brake = 0
        self.cs_lamp = 0
        self.steer_now = 0
        self.X_now = 0
        self.Y_now = 0
        self.V_now = 0
        self.yaw_now = 0 
        self.yaw_head = 0              
        self.ultra1 = 0
        self.ultra2 = 0
        self.ultra3 = 0
        self.ultra4 = 0
        self.ultra5 = 0
        self.ultra6 = 0
        self.ultra7 = 0
        self.ultra8 = 0
        # self.lidar_angle = msg.covariance[19]
        # self.lidar_range = msg.covariance[20]
        self.ZC3_to_ECU = 0 
        self.ZC2 = 0
        self.PT3 = 0
        self.ZC7 = 0
        self.ZC8 = 0
        self.ZC6 = 0
        self.ECVLAV = 0
        self.ECVLAR = 0
        self.PS4 = 0
        self.LS2 = 0
        self.PS1 = 0
        self.PDS1 = 0
        self.PDS2 = 0
        self.PS2 = 0
        self.TS1 = 0
        self.LS1 = 0
        self.autonomous_switch = 0
        self.emgs_button = 0
        self.YS31_to_PVG = 0
        self.YS32_to_PVG = 0
        self.ESV1415_to_PVG = 0
        self.ECV2J19_to_PVG = 0
        self.B01 = 0
        self.C01 = 0
        self.LS4 = 0
        self.ESVDLAR_ENA = 0
        self.ECVLAR_ENA = 0
        self.ST1 = 0


        # Inisiasi Nilai Awal
        # self.cs_lamp = 1
        self.mode_auto = 0
        self.mode_auto_bef = 0

        # # bounding box
        # self.X_utm = [558103.34,558082.38, 558082.34, 558082.34, 558103.22, 558103.30, 558103.34]
        # self.Y_utm = [9158033.03, 9158033.05, 9157996.57, 9157931.34, 9157931.32, 9157996.55, 9158033.03]
        # UTM coordinates of the bridge
        self.X_utm = [558106.622215, 558103.485361, 558100.124337, 558096.585934, 558093.201858, 558089.771423, \
                558086.438205, 558083.146216, 558083.805074, 558086.989303, 558090.473591, 558093.950218, \
                558097.354744, 558100.609169, 558103.974908, 558107.324993, 558106.622215]
        self.Y_utm = [9158033.23834, 9158033.10195, 9158033.06035, 9158033.04034, 9158032.9398,  9158032.96462, \
                9158032.96076, 9158032.99646, 9157920.25556, 9157920.20083, 9157920.32409, 9157920.34602, \
                9157920.33782, 9157920.40939, 9157920.40419, 9157920.37652, 9158033.23834]

        # first position approximation
        self.X_now = 0
        self.Y_now = 0
        self.yaw_now = 0
        # self.yawrot = np.radians(45.8)
        # self.yawrot = np.radians(45.8) - np.pi/2
        self.yawrot = 0
       
        # waypoint information
        waypoints_path = rospy.get_param('~waypoints_path', 'sigmoid220722.npy')
        waypoints_path = os.path.abspath(sys.path[0] + '/../src/waypoints/' + waypoints_path)

        # self.PSbackX = rospy.get_param('~PSback_X', -1.55*11+0.6*4)#558088.34)
        # self.PSbackY = rospy.get_param('~PSback_Y', 17)#9157952.32)
        self.PSbackX = 0
        self.PSbackY = 17

        # self.PSdockX = rospy.get_param('~dock_X', -1.55*11+0.6*4)#558088.34)
        # self.PSdockY = rospy.get_param('~dock_Y', 0)#9157941.199000001)
        self.PSdockX = 0
        self.PSdockY = 0
        self.waypoints = np.load(waypoints_path)
        self.yaw_head = self.yaw_now
        # docking spec (x,y,orientation,length,width)
        # left docking
        width_docking = 0.6

        self.l_dock_ = [-1.55, -0.5, -1.55, 14.2]
        # right docking
        self.r_dock_ = [1.55,-0.5, 1.55, 14.2]
        # lower docking
        self.b_dock_ = [-1.55,-0.5, 1.55, -0.5]

        self.l_dock_1 = [-(1.55*3+width_docking), -0.5, -(1.55*3+width_docking), 14.2]
        # right docking
        self.r_dock_1 = [-(1.55*1+width_docking),-0.5, -(1.55*1+width_docking), 14.2]
        # lower docking
        self.b_dock_1 = [-(1.55*3+width_docking),-0.5, -(1.55*1+width_docking), -0.5]

        self.l_dock_3 = [-(1.55*5+width_docking*2), -0.5, -(1.55*5+width_docking*2), 14.2]
        # right docking
        self.r_dock_3 = [-(1.55*3+width_docking*2),-0.5, -(1.55*3+width_docking*2), 14.2]
        # lower docking
        self.b_dock_3 = [-(1.55*5+width_docking*2),-0.5, -(1.55*3+width_docking*2), -0.5]

        self.l_dock_4 = [-(1.55*7+width_docking*3), -0.5, -(1.55*7+width_docking*3), 14.2]
        # right docking
        self.r_dock_4 = [-(1.55*5+width_docking*3),-0.5, -(1.55*5+width_docking*3), 14.2]
        # lower docking
        self.b_dock_4 = [-(1.55*7+width_docking*3),-0.5, -(1.55*5+width_docking*3), -0.5]

        self.l_dock_5 = [-(1.55*9+width_docking*4), -0.5, -(1.55*9+width_docking*4), 14.2]
        # right docking
        self.r_dock_5 = [-(1.55*7+width_docking*4),-0.5, -(1.55*7+width_docking*4), 14.2]
        # lower docking
        self.b_dock_5 = [-(1.55*9+width_docking*4),-0.5, -(1.55*7+width_docking*4), -0.5]

        self.l_dock_6 = [-(1.55*11+width_docking*5), -0.5, -(1.55*11+width_docking*5), 14.2]
        # right docking
        self.r_dock_6 = [-(1.55*9+width_docking*5),-0.5, -(1.55*9+width_docking*5), 14.2]
        # lower docking
        self.b_dock_6 = [-(1.55*11+width_docking*5),-0.5, -(1.55*9+width_docking*5), -0.5]

        # self.l_dock_3 = [1.55*1+width_docking, -0.5, 1.55*1+width_docking, 14.2]
        # # right docking
        # self.r_dock_3 = [1.55*3+width_docking,-0.5, 1.55*3+width_docking, 14.2]
        # # lower docking
        # self.b_dock_3 = [1.55*1+width_docking,-0.5, 1.55*3+width_docking, -0.5]

        # self.l_dock_4 = [1.55*3+width_docking*2, -0.5, 1.55*3+width_docking*2, 14.2]
        # # right docking
        # self.r_dock_4 = [1.55*5+width_docking*2,-0.5, 1.55*5+width_docking*2, 14.2]
        # # lower docking
        # self.b_dock_4 = [1.55*3+width_docking*2,-0.5, 1.55*5+width_docking*2, -0.5]

        # self.l_dock_5 = [1.55*5+width_docking*3, -0.5, 1.55*5+width_docking*3, 14.2]
        # # right docking
        # self.r_dock_5 = [1.55*7+width_docking*3,-0.5, 1.55*7+width_docking*3, 14.2]
        # # lower docking
        # self.b_dock_5 = [1.55*5+width_docking*3,-0.5, 1.55*7+width_docking*3, -0.5]

        # self.l_dock_6 = [1.55*7+width_docking*4, -0.5, 1.55*7+width_docking*4, 14.2]
        # # right docking
        # self.r_dock_6 = [1.55*9+width_docking*4,-0.5, 1.55*9+width_docking*4, 14.2]
        # # lower docking
        # self.b_dock_6 = [1.55*7+width_docking*4,-0.5, 1.55*9+width_docking*4, -0.5]

        
        # vehicle parameter
        self.l_t = 14.02
        self.w_t = 2.682
        self.l_h = 3.533
        self.w_h = 2.682
        self.V_now = 0
        self.cs_long = 0
        self.steer_now = 0
        self.cs_lat = 0

        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1280, 720)
        MainWindow.setMinimumSize(QtCore.QSize(1000, 500))
        MainWindow.setStyleSheet("background-color: rgb(45, 45, 45);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.Content = QtWidgets.QFrame(self.centralwidget)
        self.Content.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.Content.setFrameShadow(QtWidgets.QFrame.Raised)
        self.Content.setObjectName("Content")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.Content)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.frame_pages = QtWidgets.QFrame(self.Content)
        self.frame_pages.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_pages.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_pages.setObjectName("frame_pages")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.frame_pages)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.stackedWidget = QtWidgets.QStackedWidget(self.frame_pages)
        self.stackedWidget.setObjectName("stackedWidget")
        
        self.page_1_dashboard = QtWidgets.QWidget()
        self.page_1_dashboard.setObjectName("page_1_dashboard")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.page_1_dashboard)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.frame = QtWidgets.QFrame(self.page_1_dashboard)
        self.frame.setStyleSheet("background-color: rgb(77, 77, 127);")
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")

# --------------------------------- Image CTT --------------------------------------        
        self.imageCTT = QtWidgets.QLabel(self.frame)
        self.imageCTT.setGeometry(QtCore.QRect(480, 120, 101, 401))
        self.imageCTT.setAutoFillBackground(False)
        self.imageCTT.setText("")
        self.imageCTT.setScaledContents(True)
        self.imageCTT.setWordWrap(False)
        self.imageCTT.setObjectName("imageCTT")
        self.qpixmap = QPixmap('/home/agv-ctt/catkin_CTT/src/AGV_CTT/nodes/HMI Truck/truck.png')
        self.imageCTT.setPixmap(self.qpixmap)
        self.imageCTT.resize(100,375)

# --------------------------- Object Detction Dashboard -----------------------------
        self.sensorCamera = QtWidgets.QFrame(self.frame)
        self.sensorCamera.setGeometry(QtCore.QRect(514, 153, 31, 41))
        self.sensorCamera.setStyleSheet("background-color: rgb(58, 58, 102);")
        self.sensorCamera.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorCamera.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorCamera.setObjectName("sensorCamera")

        self.sensorLidar = QtWidgets.QFrame(self.frame)
        self.sensorLidar.setGeometry(QtCore.QRect(440, 80, 171, 81))
        self.sensorLidar.setStyleSheet("background-color: none;")
        self.sensorLidar.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.sensorLidar.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorLidar.setObjectName("sensorLidar")
        self.sensorLidar_detect_3 = QtWidgets.QFrame(self.sensorLidar)
        self.sensorLidar_detect_3.setGeometry(QtCore.QRect(10, 7, 153, 153))
        self.sensorLidar_detect_3.setStyleSheet("border-radius: 75px;\n"
"background-color: rgb(255, 255, 255);")
        self.sensorLidar_detect_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorLidar_detect_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorLidar_detect_3.setObjectName("sensorLidar_detect_3")
        self.sensorLidar_main_detect = QtWidgets.QFrame(self.sensorLidar_detect_3)
        self.sensorLidar_main_detect.setGeometry(QtCore.QRect(1, 1, 151, 151))
        self.sensorLidar_main_detect.setStyleSheet("border-radius: 75px;\n"
"background-color: rgb(198, 59, 52);")
        self.sensorLidar_main_detect.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorLidar_main_detect.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorLidar_main_detect.setObjectName("sensorLidar_main_detect")
        self.sensorLidar_background_2 = QtWidgets.QFrame(self.sensorLidar_main_detect)
        self.sensorLidar_background_2.setGeometry(QtCore.QRect(10, 20, 131, 131))
        self.sensorLidar_background_2.setStyleSheet("border-radius: 65px;\n"
"background-color: rgb(77, 77, 127);")
        self.sensorLidar_background_2.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.sensorLidar_background_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorLidar_background_2.setObjectName("sensorLidar_background_2")
        self.sensorLidar_background2_2 = QtWidgets.QFrame(self.sensorLidar_main_detect)
        self.sensorLidar_background2_2.setGeometry(QtCore.QRect(0, 70, 151, 86))
        self.sensorLidar_background2_2.setStyleSheet("background-color: rgba(77, 77, 127)")
        self.sensorLidar_background2_2.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.sensorLidar_background2_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorLidar_background2_2.setObjectName("sensorLidar_background2_2")

        self.sensorUltra = QtWidgets.QFrame(self.frame)
        self.sensorUltra.setGeometry(QtCore.QRect(410, 230, 241, 331))
        self.sensorUltra.setStyleSheet("background-color: none;")
        self.sensorUltra.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.sensorUltra.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorUltra.setObjectName("sensorUltra")
        self.sensorUltra_8 = QtWidgets.QFrame(self.sensorUltra)
        self.sensorUltra_8.setGeometry(QtCore.QRect(190, 0, 51, 31))
        self.sensorUltra_8.setStyleSheet("background-color: rgb(58, 58, 102);")
        self.sensorUltra_8.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorUltra_8.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorUltra_8.setObjectName("sensorUltra_8")
        self.sensorUltra_7 = QtWidgets.QFrame(self.sensorUltra)
        self.sensorUltra_7.setGeometry(QtCore.QRect(190, 100, 51, 31))
        self.sensorUltra_7.setStyleSheet("background-color: rgb(58, 58, 102);")
        self.sensorUltra_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorUltra_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorUltra_7.setObjectName("sensorUltra_7")
        self.sensorUltra_6 = QtWidgets.QFrame(self.sensorUltra)
        self.sensorUltra_6.setGeometry(QtCore.QRect(190, 180, 51, 31))
        self.sensorUltra_6.setStyleSheet("background-color: rgb(58, 58, 102);")
        self.sensorUltra_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorUltra_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorUltra_6.setObjectName("sensorUltra_6")
        self.sensorUltra_1 = QtWidgets.QFrame(self.sensorUltra)
        self.sensorUltra_1.setGeometry(QtCore.QRect(0, 0, 51, 31))
        self.sensorUltra_1.setStyleSheet("background-color: rgb(58, 58, 102);")
        self.sensorUltra_1.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorUltra_1.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorUltra_1.setObjectName("sensorUltra_1")
        self.sensorUltra_2 = QtWidgets.QFrame(self.sensorUltra)
        self.sensorUltra_2.setGeometry(QtCore.QRect(0, 100, 51, 31))
        self.sensorUltra_2.setStyleSheet("background-color: rgb(58, 58, 102);")
        self.sensorUltra_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorUltra_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorUltra_2.setObjectName("sensorUltra_2")
        self.sensorUltra_3 = QtWidgets.QFrame(self.sensorUltra)
        self.sensorUltra_3.setGeometry(QtCore.QRect(0, 180, 51, 31))
        self.sensorUltra_3.setStyleSheet("background-color: rgb(58, 58, 102);")
        self.sensorUltra_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorUltra_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorUltra_3.setObjectName("sensorUltra_3")
        self.sensorUltra_4 = QtWidgets.QFrame(self.sensorUltra)
        self.sensorUltra_4.setGeometry(QtCore.QRect(50, 300, 51, 31))
        self.sensorUltra_4.setStyleSheet("background-color: rgb(58, 58, 102);")
        self.sensorUltra_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorUltra_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorUltra_4.setObjectName("sensorUltra_4")
        self.sensorUltra_5 = QtWidgets.QFrame(self.sensorUltra)
        self.sensorUltra_5.setGeometry(QtCore.QRect(140, 300, 51, 31))
        self.sensorUltra_5.setStyleSheet("background-color: rgb(58, 58, 102);")
        self.sensorUltra_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorUltra_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorUltra_5.setObjectName("sensorUltra_5")

# -------------------------- Indicator Forward/Backward/Neutral ----------------------        
        self.switch_move = QtWidgets.QFrame(self.frame)
        self.switch_move.setGeometry(QtCore.QRect(435, 570, 191, 61))
        self.switch_move.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.switch_move.setFrameShadow(QtWidgets.QFrame.Raised)
        self.switch_move.setObjectName("switch_move")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.switch_move)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.switch_netral_2 = QtWidgets.QLabel(self.switch_move)
        self.switch_netral_2.setEnabled(True)
        font = QtGui.QFont()
        font.setFamily("Noto Sans Telugu UI")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.switch_netral_2.setFont(font)
        self.switch_netral_2.setStyleSheet("color: rgb(85, 255, 127);")
        self.switch_netral_2.setFrameShape(QtWidgets.QFrame.Box)
        self.switch_netral_2.setAlignment(QtCore.Qt.AlignCenter)
        self.switch_netral_2.setObjectName("switch_netral_2")
        self.horizontalLayout_4.addWidget(self.switch_netral_2)
        self.switch_otonom = QtWidgets.QFrame(self.frame)
        self.switch_otonom.setGeometry(QtCore.QRect(430, 30, 191, 41))
        self.switch_otonom.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.switch_otonom.setFrameShadow(QtWidgets.QFrame.Raised)
        self.switch_otonom.setObjectName("switch_otonom")
        self.label_switch_otonom = QtWidgets.QLabel(self.switch_otonom)
        self.label_switch_otonom.setGeometry(QtCore.QRect(10, 10, 171, 20))
        font = QtGui.QFont()
        font.setFamily("Noto Sans Georgian")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_switch_otonom.setFont(font)
        self.label_switch_otonom.setStyleSheet("color:rgb(255, 255, 255)")
        self.label_switch_otonom.setAlignment(QtCore.Qt.AlignCenter)
        self.label_switch_otonom.setObjectName("label_switch_otonom")

# --------------------------------- Mathplotlib ---------------------------------------------------------
        self.mathplotlib = QtWidgets.QFrame(self.frame)
        self.mathplotlib.setGeometry(QtCore.QRect(30, 40, 311, 581))
        self.mathplotlib.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.mathplotlib.setFrameShadow(QtWidgets.QFrame.Raised)
        self.mathplotlib.setObjectName("mathplotlib")
        # Mathplotlib
        super().__init__()
        self.layout = QtWidgets.QVBoxLayout(self.mathplotlib)
        dynamic_canvas = FigureCanvas(Figure(figsize=(25, 25)))
        self.layout.addWidget(dynamic_canvas)
        self._dynamic_ax = dynamic_canvas.figure.subplots()
        self.plot_init()
        # timer and call back every 20 ms
        self._timer = dynamic_canvas.new_timer(20)
        self._timer.add_callback(self._update_canvas)
        self._timer.start()

# ------------------------------ Break in Dashboard ---------------------------------------------------------------
        self.circular_feedBrake = QtWidgets.QFrame(self.frame)
        self.circular_feedBrake.setGeometry(QtCore.QRect(680, 230, 181, 181))
        self.circular_feedBrake.setStyleSheet("background-color: none;")
        self.circular_feedBrake.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.circular_feedBrake.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circular_feedBrake.setObjectName("circular_feedBrake")
        self.circularBg_feedBrake = QtWidgets.QFrame(self.circular_feedBrake)
        self.circularBg_feedBrake.setGeometry(QtCore.QRect(10, 10, 161, 161))
        self.circularBg_feedBrake.setStyleSheet("QFrame{\n"
"    border-radius: 80px;    \n"
"    background-color: rgba(85, 85, 127, 100);\n"
"}")
        self.circularBg_feedBrake.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.circularBg_feedBrake.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circularBg_feedBrake.setObjectName("circularBg_feedBrake")
        self.animasi_feedBrake = QtWidgets.QFrame(self.circularBg_feedBrake)
        self.animasi_feedBrake.setGeometry(QtCore.QRect(0, 0, 161, 161))
        self.animasi_feedBrake.setStyleSheet("QFrame{\n"
"    border-radius: 80px;    \n"
"    background-color: qconicalgradient(cx:0.5, cy:0.5, angle:90, stop:0.750 rgb(0, 255, 7), stop:0.745 rgba(255, 255, 255, 0));\n"
"}")
        self.animasi_feedBrake.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.animasi_feedBrake.setFrameShadow(QtWidgets.QFrame.Raised)
        self.animasi_feedBrake.setObjectName("animasi_feedBrake")
        self.circularContainer_feedBrake = QtWidgets.QFrame(self.circular_feedBrake)
        self.circularContainer_feedBrake.setGeometry(QtCore.QRect(25, 25, 131, 131))
        self.circularContainer_feedBrake.setBaseSize(QtCore.QSize(0, 0))
        self.circularContainer_feedBrake.setStyleSheet("QFrame{\n"
"    border-radius: 65px;    \n"
"    background-color: rgb(58, 58, 102);\n"
"}")
        self.circularContainer_feedBrake.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.circularContainer_feedBrake.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circularContainer_feedBrake.setObjectName("circularContainer_feedBrake")
        self.layoutWidget_4 = QtWidgets.QWidget(self.circularContainer_feedBrake)
        self.layoutWidget_4.setGeometry(QtCore.QRect(10, 20, 111, 101))
        self.layoutWidget_4.setObjectName("layoutWidget_4")
        self.infoLayout_feedBrake = QtWidgets.QGridLayout(self.layoutWidget_4)
        self.infoLayout_feedBrake.setContentsMargins(0, 0, 0, 0)
        self.infoLayout_feedBrake.setObjectName("infoLayout_feedBrake")
        self.label_feedBrake_2 = QtWidgets.QLabel(self.layoutWidget_4)
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(10)
        self.label_feedBrake_2.setFont(font)
        self.label_feedBrake_2.setStyleSheet("color: #FFFFFF; background-color: none;")
        self.label_feedBrake_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_feedBrake_2.setObjectName("label_feedBrake_2")
        self.infoLayout_feedBrake.addWidget(self.label_feedBrake_2, 0, 0, 1, 1)
        self.label_progress_feedBrake = QtWidgets.QLabel(self.layoutWidget_4)
        font = QtGui.QFont()
        font.setFamily("Roboto Thin")
        font.setPointSize(30)
        self.label_progress_feedBrake.setFont(font)
        self.label_progress_feedBrake.setStyleSheet("color: rgb(0, 255, 7);; padding: 0px; background-color: none;\n"
"")
        self.label_progress_feedBrake.setAlignment(QtCore.Qt.AlignCenter)
        self.label_progress_feedBrake.setIndent(-1)
        self.label_progress_feedBrake.setObjectName("label_progress_feedBrake")
        self.infoLayout_feedBrake.addWidget(self.label_progress_feedBrake, 1, 0, 1, 1)
        self.label_feedBrake = QtWidgets.QLabel(self.layoutWidget_4)
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(14)
        self.label_feedBrake.setFont(font)
        self.label_feedBrake.setStyleSheet("color: rgb(148, 148, 216); background-color: none;")
        self.label_feedBrake.setAlignment(QtCore.Qt.AlignCenter)
        self.label_feedBrake.setObjectName("label_feedBrake")
        self.infoLayout_feedBrake.addWidget(self.label_feedBrake, 2, 0, 1, 1)

# ----------------------------- Speed in Dashboard ------------------------------
        self.circular_feedspeed = QtWidgets.QFrame(self.frame)
        self.circular_feedspeed.setGeometry(QtCore.QRect(680, 40, 181, 181))
        self.circular_feedspeed.setStyleSheet("background-color: none;")
        self.circular_feedspeed.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.circular_feedspeed.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circular_feedspeed.setObjectName("circular_feedspeed")
        self.circularBg_feedspeed = QtWidgets.QFrame(self.circular_feedspeed)
        self.circularBg_feedspeed.setGeometry(QtCore.QRect(10, 10, 161, 161))
        self.circularBg_feedspeed.setStyleSheet("QFrame{\n"
"    border-radius: 80px;    \n"
"    background-color: rgba(85, 85, 127, 100);\n"
"}")
        self.circularBg_feedspeed.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.circularBg_feedspeed.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circularBg_feedspeed.setObjectName("circularBg_feedspeed")
        self.animasi_feedspeed = QtWidgets.QFrame(self.circularBg_feedspeed)
        self.animasi_feedspeed.setGeometry(QtCore.QRect(0, 0, 161, 161))
        self.animasi_feedspeed.setStyleSheet("QFrame{\n"
"    border-radius: 80px;    \n"
"    background-color: qconicalgradient(cx:0.5, cy:0.5, angle:90, stop:0.750 rgba(255, 0, 127, 255), stop:0.745 rgba(255, 255, 255, 0));\n"
"}")
        self.animasi_feedspeed.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.animasi_feedspeed.setFrameShadow(QtWidgets.QFrame.Raised)
        self.animasi_feedspeed.setObjectName("animasi_feedspeed")
        self.circularContainer_feedspeed = QtWidgets.QFrame(self.circular_feedspeed)
        self.circularContainer_feedspeed.setGeometry(QtCore.QRect(25, 25, 131, 131))
        self.circularContainer_feedspeed.setBaseSize(QtCore.QSize(0, 0))
        self.circularContainer_feedspeed.setStyleSheet("QFrame{\n"
"    border-radius: 65px;    \n"
"    background-color: rgb(58, 58, 102);\n"
"}")
        self.circularContainer_feedspeed.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.circularContainer_feedspeed.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circularContainer_feedspeed.setObjectName("circularContainer_feedspeed")
        self.layoutWidget_3 = QtWidgets.QWidget(self.circularContainer_feedspeed)
        self.layoutWidget_3.setGeometry(QtCore.QRect(10, 20, 111, 102))
        self.layoutWidget_3.setObjectName("layoutWidget_3")
        self.infoLayout_feedspeed = QtWidgets.QGridLayout(self.layoutWidget_3)
        self.infoLayout_feedspeed.setContentsMargins(0, 0, 0, 0)
        self.infoLayout_feedspeed.setObjectName("infoLayout_feedspeed")
        self.label_feedspeed_2 = QtWidgets.QLabel(self.layoutWidget_3)
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(10)
        self.label_feedspeed_2.setFont(font)
        self.label_feedspeed_2.setStyleSheet("color: #FFFFFF; background-color: none;")
        self.label_feedspeed_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_feedspeed_2.setObjectName("label_feedspeed_2")
        self.infoLayout_feedspeed.addWidget(self.label_feedspeed_2, 0, 0, 1, 1)
        self.label_progress_feedspeed = QtWidgets.QLabel(self.layoutWidget_3)
        font = QtGui.QFont()
        font.setFamily("Roboto Thin")
        font.setPointSize(30)
        self.label_progress_feedspeed.setFont(font)
        self.label_progress_feedspeed.setStyleSheet("color: rgb(255, 44, 174); padding: 0px; background-color: none;")
        self.label_progress_feedspeed.setAlignment(QtCore.Qt.AlignCenter)
        self.label_progress_feedspeed.setIndent(-1)
        self.label_progress_feedspeed.setObjectName("label_progress_feedspeed")
        self.infoLayout_feedspeed.addWidget(self.label_progress_feedspeed, 1, 0, 1, 1)
        self.label_feedspeed = QtWidgets.QLabel(self.layoutWidget_3)
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(14)
        self.label_feedspeed.setFont(font)
        self.label_feedspeed.setStyleSheet("color: rgb(148, 148, 216); background-color: none;")
        self.label_feedspeed.setAlignment(QtCore.Qt.AlignCenter)
        self.label_feedspeed.setObjectName("label_feedspeed")
        self.infoLayout_feedspeed.addWidget(self.label_feedspeed, 2, 0, 1, 1)

# ----------------------------- Steer in Dashboard -----------------------------
        self.circular_feedsteer = QtWidgets.QFrame(self.frame)
        self.circular_feedsteer.setGeometry(QtCore.QRect(680, 420, 181, 181))
        self.circular_feedsteer.setMaximumSize(QtCore.QSize(10000, 10000))
        self.circular_feedsteer.setStyleSheet("background-color: none;")
        self.circular_feedsteer.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.circular_feedsteer.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circular_feedsteer.setObjectName("circular_feedsteer")
        self.circularBg_feedsteer = QtWidgets.QFrame(self.circular_feedsteer)
        self.circularBg_feedsteer.setGeometry(QtCore.QRect(10, 10, 161, 161))
        self.circularBg_feedsteer.setStyleSheet("QFrame{\n"
"    border-radius: 80px;    \n"
"    background-color: rgba(85, 85, 127, 100);\n"
"}")
        self.circularBg_feedsteer.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.circularBg_feedsteer.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circularBg_feedsteer.setObjectName("circularBg_feedsteer")
        self.animasi_feedsteer = QtWidgets.QFrame(self.circularBg_feedsteer)
        self.animasi_feedsteer.setGeometry(QtCore.QRect(0, 0, 161, 161))
        self.animasi_feedsteer.setStyleSheet("QFrame{\n"
"    border-radius: 80px;\n"
"    border-strokes:inside;    \n"
"    background-color: qconicalgradient(cx:0.5, cy:0.5, angle:90, stop:0.400 rgba(85, 170, 255, 255), stop:0.395 rgba(255, 255, 255, 0));\n"
"}")
        self.animasi_feedsteer.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.animasi_feedsteer.setFrameShadow(QtWidgets.QFrame.Raised)
        self.animasi_feedsteer.setObjectName("animasi_feedsteer")
        self.circularContainer_feedsteer = QtWidgets.QFrame(self.circular_feedsteer)
        self.circularContainer_feedsteer.setGeometry(QtCore.QRect(25, 25, 131, 131))
        self.circularContainer_feedsteer.setBaseSize(QtCore.QSize(0, 0))
        self.circularContainer_feedsteer.setStyleSheet("QFrame{\n"
"    border-radius: 65px;    \n"
"    background-color: rgb(58, 58, 102);\n"
"}")
        self.circularContainer_feedsteer.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.circularContainer_feedsteer.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circularContainer_feedsteer.setObjectName("circularContainer_feedsteer")
        self.layoutWidget = QtWidgets.QWidget(self.circularContainer_feedsteer)
        self.layoutWidget.setGeometry(QtCore.QRect(10, 20, 111, 101))
        self.layoutWidget.setObjectName("layoutWidget")
        self.infoLayout_feedsteer = QtWidgets.QGridLayout(self.layoutWidget)
        self.infoLayout_feedsteer.setContentsMargins(0, 0, 0, 0)
        self.infoLayout_feedsteer.setObjectName("infoLayout_feedsteer")
        self.label_feedsteer_2 = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(10)
        self.label_feedsteer_2.setFont(font)
        self.label_feedsteer_2.setStyleSheet("color: #FFFFFF; background-color: none;")
        self.label_feedsteer_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_feedsteer_2.setObjectName("label_feedsteer_2")
        self.infoLayout_feedsteer.addWidget(self.label_feedsteer_2, 0, 0, 1, 1)
        self.label_progress_feedsteer = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setFamily("Roboto Thin")
        font.setPointSize(30)
        self.label_progress_feedsteer.setFont(font)
        self.label_progress_feedsteer.setStyleSheet("color: rgb(115, 185, 255); padding: 0px; background-color: none;")
        self.label_progress_feedsteer.setAlignment(QtCore.Qt.AlignCenter)
        self.label_progress_feedsteer.setIndent(-1)
        self.label_progress_feedsteer.setObjectName("label_progress_feedsteer")
        self.infoLayout_feedsteer.addWidget(self.label_progress_feedsteer, 1, 0, 1, 1)
        self.label_feedsteer = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(14)
        self.label_feedsteer.setFont(font)
        self.label_feedsteer.setStyleSheet("color: rgb(148, 148, 216); background-color: none;")
        self.label_feedsteer.setAlignment(QtCore.Qt.AlignCenter)
        self.label_feedsteer.setObjectName("label_feedsteer")
        self.infoLayout_feedsteer.addWidget(self.label_feedsteer, 2, 0, 1, 1)

# -------------------------------- Autonomous Mode Button -----------------------------
        self.frame_buttonAuto = QtWidgets.QFrame(self.frame)
        self.frame_buttonAuto.setGeometry(QtCore.QRect(910, 30, 231, 301))
        self.frame_buttonAuto.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_buttonAuto.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_buttonAuto.setObjectName("frame_buttonAuto")
        self.label_buttonAuto = QtWidgets.QLabel(self.frame_buttonAuto)
        self.label_buttonAuto.setGeometry(QtCore.QRect(49, 12, 141, 17))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_buttonAuto.setFont(font)
        self.label_buttonAuto.setStyleSheet("color: rgb(255, 255, 255);")
        self.label_buttonAuto.setAlignment(QtCore.Qt.AlignCenter)
        self.label_buttonAuto.setObjectName("label_buttonAuto")

        self.button_ps = QtWidgets.QPushButton(self.frame_buttonAuto)
        self.button_ps.setGeometry(QtCore.QRect(20, 40, 191, 41))
        self.button_ps.setStyleSheet("QPushButton {\n"
"    color: rgb(255, 255, 255);\n"
"    background-color: rgb(35, 35, 35);\n"
"    border: 0px solid;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(85, 170, 255);\n"
"}")
        self.button_ps.setObjectName("button_ps")
        self.button_ps.clicked.connect(self.mode4)

        self.button_dock = QtWidgets.QPushButton(self.frame_buttonAuto)
        self.button_dock.setGeometry(QtCore.QRect(20, 90, 191, 41))
        self.button_dock.setStyleSheet("QPushButton {\n"
"    color: rgb(255, 255, 255);\n"
"    background-color: rgb(35, 35, 35);\n"
"    border: 0px solid;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(85, 170, 255);\n"
"}")
        self.button_dock.setObjectName("button_dock")
        self.button_dock.clicked.connect(self.mode5)

        self.button_takeplace = QtWidgets.QPushButton(self.frame_buttonAuto)
        self.button_takeplace.setGeometry(QtCore.QRect(20, 140, 191, 41))
        self.button_takeplace.setStyleSheet("QPushButton {\n"
"    color: rgb(255, 255, 255);\n"
"    background-color: rgb(35, 35, 35);\n"
"    border: 0px solid;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(85, 170, 255);\n"
"}")
        self.button_takeplace.setObjectName("button_takeplace")
        self.button_takeplace.clicked.connect(self.mode_takeplace)

        self.button_undock = QtWidgets.QPushButton(self.frame_buttonAuto)
        self.button_undock.setGeometry(QtCore.QRect(20, 190, 191, 41))
        self.button_undock.setStyleSheet("QPushButton {\n"
"    color: rgb(255, 255, 255);\n"
"    background-color: rgb(35, 35, 35);\n"
"    border: 0px solid;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(85, 170, 255);\n"
"}")
        self.button_undock.setObjectName("button_undock")
        self.button_undock.clicked.connect(self.mode_undocking)

        self.button_path = QtWidgets.QPushButton(self.frame_buttonAuto)
        self.button_path.setGeometry(QtCore.QRect(20, 240, 191, 41))
        self.button_path.setStyleSheet("QPushButton {\n"
"    color: rgb(255, 255, 255);\n"
"    background-color: rgb(35, 35, 35);\n"
"    border: 0px solid;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(85, 170, 255);\n"
"}")
        self.button_path.setObjectName("button_path")
        self.button_path.clicked.connect(self.mode3)

        self.frame_buttonMaintance = QtWidgets.QFrame(self.frame)
        self.frame_buttonMaintance.setGeometry(QtCore.QRect(910, 350, 231, 301))
        self.frame_buttonMaintance.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_buttonMaintance.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_buttonMaintance.setObjectName("frame_buttonMaintance")
        self.label_buttonMaintance = QtWidgets.QLabel(self.frame_buttonMaintance)
        self.label_buttonMaintance.setGeometry(QtCore.QRect(40, 10, 151, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_buttonMaintance.setFont(font)
        self.label_buttonMaintance.setStyleSheet("color: rgb(255, 255, 255);")
        self.label_buttonMaintance.setAlignment(QtCore.Qt.AlignCenter)
        self.label_buttonMaintance.setObjectName("label_buttonMaintance")

        self.button_check_liftingUP = QtWidgets.QPushButton(self.frame_buttonMaintance)
        self.button_check_liftingUP.setGeometry(QtCore.QRect(20, 40, 191, 41))
        self.button_check_liftingUP.setStyleSheet("QPushButton {\n"
"    color: rgb(255, 255, 255);\n"
"    background-color: rgb(35, 35, 35);\n"
"    border: 0px solid;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(85, 170, 255);\n"
"}")
        self.button_check_liftingUP.setObjectName("button_check_liftingUP")
        self.button_check_liftingUP.clicked.connect(self.mode_liftUp)

        self.button_check_liftingMid = QtWidgets.QPushButton(self.frame_buttonMaintance)
        self.button_check_liftingMid.setGeometry(QtCore.QRect(20, 90, 191, 41))
        self.button_check_liftingMid.setStyleSheet("QPushButton {\n"
"    color: rgb(255, 255, 255);\n"
"    background-color: rgb(35, 35, 35);\n"
"    border: 0px solid;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(85, 170, 255);\n"
"}")
        self.button_check_liftingMid.setObjectName("button_check_liftingMid")
        self.button_check_liftingMid.clicked.connect(self.mode_liftMid)        

        self.button_check_liftingDown = QtWidgets.QPushButton(self.frame_buttonMaintance)
        self.button_check_liftingDown.setGeometry(QtCore.QRect(20, 140, 191, 41))
        self.button_check_liftingDown.setStyleSheet("QPushButton {\n"
"    color: rgb(255, 255, 255);\n"
"    background-color: rgb(35, 35, 35);\n"
"    border: 0px solid;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(85, 170, 255);\n"
"}")
        self.button_check_liftingDown.setObjectName("button_check_liftingDown")
        self.button_check_liftingDown.clicked.connect(self.mode_liftDown)

        self.button_check_steer = QtWidgets.QPushButton(self.frame_buttonMaintance)
        self.button_check_steer.setGeometry(QtCore.QRect(20, 190, 191, 41))
        self.button_check_steer.setStyleSheet("QPushButton {\n"
"    color: rgb(255, 255, 255);\n"
"    background-color: rgb(35, 35, 35);\n"
"    border: 0px solid;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(85, 170, 255);\n"
"}")
        self.button_check_steer.setObjectName("button_check_steer")
        self.button_check_steer.clicked.connect(self.mode1)

        self.button_check_throttle = QtWidgets.QPushButton(self.frame_buttonMaintance)
        self.button_check_throttle.setGeometry(QtCore.QRect(20, 240, 191, 41))
        self.button_check_throttle.setStyleSheet("QPushButton {\n"
"    color: rgb(255, 255, 255);\n"
"    background-color: rgb(35, 35, 35);\n"
"    border: 0px solid;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(85, 170, 255);\n"
"}")
        self.button_check_throttle.setObjectName("button_check_throttle")
        self.button_check_throttle.clicked.connect(self.mode2)

# ---------------------------- Next Page ----------------------------------
        self.button_nextPage = QtWidgets.QPushButton(self.frame)
        self.button_nextPage.setGeometry(QtCore.QRect(1200, 320, 51, 41))
        self.button_nextPage.setStyleSheet("QPushButton {\n"
"    color: rgb(255, 255, 255);\n"
"    background-color: rgb(35, 35, 35);\n"
"    border: 0px solid;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(85, 170, 255);\n"
"}")
        self.button_nextPage.setObjectName("button_nextPage")

        self.frame_buttonAuto.raise_()
        self.sensorLidar.raise_()
        self.switch_move.raise_()
        self.switch_otonom.raise_()
        self.mathplotlib.raise_()
        self.sensorUltra.raise_()
        self.imageCTT.raise_()
        self.circular_feedBrake.raise_()
        self.circular_feedspeed.raise_()
        self.circular_feedsteer.raise_()
        self.sensorCamera.raise_()
        self.frame_buttonMaintance.raise_()
        self.button_nextPage.raise_()
        self.verticalLayout_7.addWidget(self.frame)
        self.stackedWidget.addWidget(self.page_1_dashboard)
        self.verticalLayout_5.addWidget(self.stackedWidget)
        self.horizontalLayout_2.addWidget(self.frame_pages)
        self.verticalLayout.addWidget(self.Content)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        self.stackedWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.switch_netral_2.setText(_translate("MainWindow", " BACKWARD"))
        self.label_switch_otonom.setText(_translate("MainWindow", "AUTONOMOUS"))
        self.label_feedBrake_2.setText(_translate("MainWindow", "<html><head/><body><p>BRAKE</p></body></html>"))
        self.label_progress_feedBrake.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:24pt;\">25</span></p></body></html>"))
        self.label_feedBrake.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt;\">BAR</span></p></body></html>"))
        self.label_feedspeed_2.setText(_translate("MainWindow", "<html><head/><body><p>SPEED</p></body></html>"))
        self.label_progress_feedspeed.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:28pt;\">25</span></p></body></html>"))
        self.label_feedspeed.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt;\">KM/H</span></p></body></html>"))
        self.label_feedsteer_2.setText(_translate("MainWindow", "<html><head/><body><p>STEERING</p></body></html>"))
        self.label_progress_feedsteer.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:28pt;\">60</span></p></body></html>"))
        self.label_feedsteer.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt;\">DEGREE</span></p></body></html>"))
        self.button_dock.setText(_translate("MainWindow", "DOCKING"))
        self.button_ps.setText(_translate("MainWindow", "POINT STABILIZATION"))
        self.button_path.setText(_translate("MainWindow", "PATH FOLLOWING"))
        self.label_buttonAuto.setText(_translate("MainWindow", "AUTONOMOUS"))
        self.button_takeplace.setText(_translate("MainWindow", "TAKE/PLACE CONTAINER"))
        self.button_undock.setText(_translate("MainWindow", "UNDOCKING"))
        self.button_check_liftingUP.setText(_translate("MainWindow", "LIFTING UP"))
        self.button_check_liftingMid.setText(_translate("MainWindow", "LIFTING MID"))
        self.button_check_liftingDown.setText(_translate("MainWindow", "LIFTING DOWN"))
        self.label_buttonMaintance.setText(_translate("MainWindow", "MAINTANCE"))
        self.button_check_steer.setText(_translate("MainWindow", "CHECK STEER"))
        self.button_check_throttle.setText(_translate("MainWindow", "CHECK THROTTLE"))
        self.button_nextPage.setText(_translate("MainWindow", "2"))

    def callback_data_collector(self, msg):
        # ----------------------------------------- Sensor Ultrasonik --------------------------    
        self.cs_lat = msg.covariance[0]    #steering angle in rad
        self.cs_long = msg.covariance[1]   #speed set point in m/s
        self.cs_brake = msg.covariance[2]  #braking percentage
        self.cs_lamp = msg.covariance[3]   #autonomous lamp
        self.steer_now = msg.covariance[4] #steer in degree
        self.X_now = msg.covariance[5]     #sensor_data['y_rear'] 
        self.Y_now = msg.covariance[6]
        self.V_now = msg.covariance[7]
        self.yaw_now = msg.covariance[8] 
        self.yaw_head = msg.covariance[9]                
        self.ultra1 = msg.covariance[11]
        self.ultra2 = msg.covariance[12]
        self.ultra3 = msg.covariance[13]
        self.ultra4 = msg.covariance[14]
        # print(self.ultra4)
        self.ultra5 = msg.covariance[15]
        self.ultra6 = msg.covariance[16]
        self.ultra7 = msg.covariance[17]
        self.ultra8 = msg.covariance[18]

        # self.ob_ultra1.display(self.ultra1)
        # self.ob_ultra2.display(self.ultra2)
        # self.ob_ultra3.display(self.ultra3)
        # self.ob_ultra4.display(self.ultra4)
        # self.ob_ultra5.display(self.ultra5)
        # self.ob_ultra6.display(self.ultra6)
        # self.ob_ultra7.display(self.ultra7)
        # self.ob_ultra8.display(self.ultra8)

        # self.lidar_angle = msg.covariance[19]
        # self.lidar_range = msg.covariance[20]
        self.lidar_danger_flag = msg.covariance[21]

        if self.ultra1 < 40:
                self.sensorUltra_1.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_1.setStyleSheet("background-color: rgb(58, 58, 102);")     

        if self.ultra2 < 40:
                self.sensorUltra_2.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_2.setStyleSheet("background-color: rgb(58, 58, 102);")

        if self.ultra3 < 40:
                self.sensorUltra_3.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_3.setStyleSheet("background-color: rgb(58, 58, 102);")        

        if self.ultra4 < 40:
                self.sensorUltra_4.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_4.setStyleSheet("background-color: rgb(58, 58, 102);")

        if self.ultra5 < 40:
                self.sensorUltra_5.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_5.setStyleSheet("background-color: rgb(58, 58, 102);")

        if self.ultra6 < 40:
                self.sensorUltra_6.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_6.setStyleSheet("background-color: rgb(58, 58, 102);")

        if self.ultra7 < 40:
                self.sensorUltra_7.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_7.setStyleSheet("background-color: rgb(58, 58, 102);")

        if self.ultra8 < 40:
                self.sensorUltra_8.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_8.setStyleSheet("background-color: rgb(58, 58, 102);")
        
        # -------------------------------- Sensor Lidar ---------------------------------
        if self.lidar_danger_flag == 1:
                self.sensorLidar_main_detect.setStyleSheet(("border-radius: 75px;\n"
"background-color: rgb(198, 59, 52);"))
        else:
                self.sensorLidar_main_detect.setStyleSheet(("border-radius: 75px;\n"
"background-color: rgb(58, 58, 102);"))

        self.ZC3_to_ECU = msg.covariance[22] 
        self.ZC2 = msg.covariance[23]
        self.PT3 = msg.covariance[24]
        self.ZC7 = msg.covariance[25]
        self.ZC8 = msg.covariance[26]
        self.ZC6 = msg.covariance[27]
        self.ECVLAV = msg.covariance[28]
        self.ECVLAR = msg.covariance[29]
        self.PS4 = msg.covariance[30]
        self.LS2 = msg.covariance[31]
        self.PS1 = msg.covariance[32]
        self.PDS1 = msg.covariance[33]
        self.PDS2 = msg.covariance[34]
        self.PS2 = msg.covariance[35]
        self.TS1 = msg.covariance[36]
        self.LS1 = msg.covariance[37]
        self.autonomous_switch = msg.covariance[38]
        self.emgs_button = msg.covariance[39]
        self.YS31_to_PVG = msg.covariance[40]
        self.YS32_to_PVG = msg.covariance[41]
        self.ESV1415_to_PVG = msg.covariance[42]
        self.ECV2J19_to_PVG = msg.covariance[43]
        self.B01 = msg.covariance[44]
        self.C01 = msg.covariance[45]
        self.LS4 = msg.covariance[46]
        self.ESVDLAR_ENA = msg.covariance[47]
        self.ECVLAR_ENA = msg.covariance[48]
        self.ST1 = msg.covariance[49]

        # Autonomous and direction indicator
        if self.cs_lamp == 1: #autonomous indicator
            self.label_switch_otonom.setText("AUTONOMOUS")
        else: 
            self.label_switch_otonom.setText("MANUAL")

        if self.YS31_to_PVG and not self.YS32_to_PVG: #reverse
            self.switch_netral_2.setText("BACKWARD")
        elif not self.YS31_to_PVG and self.YS32_to_PVG: #forward
            self.switch_netral_2.setText("FORWARD")
        elif not self.YS31_to_PVG and not self.YS32_to_PVG: #neutral
            self.switch_netral_2.setText("NEUTRAL") 

        if self.mode_auto == 1: self.mode1()
        elif self.mode_auto == 2: self.mode2()
        elif self.mode_auto == 3: self.mode3()
        elif self.mode_auto == 4: self.mode4()
        elif self.mode_auto == 5: self.mode5()
        elif self.mode_auto == 6: self.mode_liftUp()
        elif self.mode_auto == 7: self.mode_liftMid()
        # elif self.mode_auto == 3: self.mode_liftMid()
        elif self.mode_auto == 8: self.mode_liftDown()
        elif self.mode_auto == 9: self.mode_undocking()
        elif self.mode_auto == 10: self.mode_takeplace()
        elif self.mode_auto == 0: self.mode0()
        self.display_dash()
        self.idc_lamp()

    def callback_data_conv(self, msg): ## UNUSED
        self.steer_now = msg.pose.covariance[1]

        if self.mode_auto == 1: self.mode1()
        elif self.mode_auto == 2: self.mode2()
        elif self.mode_auto == 3: self.mode3()
        elif self.mode_auto == 4: self.mode4()
        elif self.mode_auto == 5: self.mode5()
        elif self.mode_auto == 6: self.mode_liftUp()
        elif self.mode_auto == 7: self.mode_liftMid()
        # elif self.mode_auto == 3: self.mode_liftMid()
        elif self.mode_auto == 8: self.mode_liftDown()
        elif self.mode_auto == 9: self.mode_undocking()
        elif self.mode_auto == 10: self.mode_takeplace()
        elif self.mode_auto == 0: self.mode0()
        self.display_dash()

    def idc_lamp(self):            
        self.msg.pose.covariance[0] = self.mode_auto
        self.pub.publish(self.msg)

    def display_dash(self):
        # self.callback_data_collector()
        self.label_progress_feedBrake.setText(str(round(self.PT3)))
        self.label_progress_feedspeed.setText(str(round(self.V_now*3.6,1)))     #Speed
        self.label_progress_feedsteer.setText(str(round(self.steer_now,1)))     #Steering

# --------------------------------------- Animasi Steering ---------------------------------------------------       
    def animasi(self):
        # self.callback_data_collector()            
        # Stylesheet
        self.stylesheet_steer = """
        QFrame{
                border-radius: 110px;
                border-strokes:inside;
                background-color: qconicalgradient(cx:0.5, cy:0.5, angle:90, stop:{STOP_1_steer} rgba(85, 170, 255, 255), stop:{STOP_2_steer} rgba(255, 255, 255, 0));                
        }
        """
        # self.value_steer = self.cs_lat
        self.value_steer = self.cs_lat
        self.progress_steer = (45 - self.value_steer)/45.0
        
        # New Value
        self.stop_1_steer = str(self.progress_steer - 0.001)
        self.stop_2_steer = str(self.progress_steer)

        # New Stylesheet
        self.new_animasi_steer = self.stylesheet_steer.replace("{STOP_1_steer}", self.stop_1_steer).replace("{STOP_2_steer}", self.stop_2_steer)

        # Apply stylesheet
        # self.animasi_feedsteer.setStyleSheet(self.new_animasi_steer)

                # ------------------------ Animasi Speed ---------------------------------------------------       
        # Stylesheet
        self.stylesheet_speed = """
        QFrame{
                border-radius: 110px;
                border-strokes:inside;
                background-color: qconicalgradient(cx:0.5, cy:0.5, angle:90, stop:{STOP_1_speed} rgba(255, 0, 127, 255), stop:{STOP_2_speed} rgba(255, 255, 255, 0));
        }
        """
        # self.value_speed = self.cs_long
        self.value_speed = self.cs_long
        self.progress_speed = (3.6 - self.value_speed)/3.6
        
        # New Value
        self.stop_1_speed = str(self.progress_speed - 0.001)
        self.stop_2_speed = str(self.progress_speed)

        # New Stylesheet
        self.new_animasi_speed = self.stylesheet_speed.replace("{STOP_1_speed}", self.stop_1_speed).replace("{STOP_2_speed}", self.stop_2_speed)

        # Apply stylesheet
        # self.animasi_feedspeed.setStyleSheet(self.new_animasi_speed)

# ------------------------------------- Button Autonomous --------------------------------------
    def mode0(self):       #MODE0 = Netral
        self.mode_auto = 0
        self.mode_auto_bef = self.mode_auto
        self.counter = 0
        # self.label_switch_otonom.setText("MANUAL")
        # self.switch_netral_2.setText("NEUTRAL")
   
    def mode1(self): #MODE1: Steer Check
        # check state before
        # print("masuk steer")
        if self.mode_auto_bef == 0: 
            self.mode_auto = 1
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        self.counter += 1 
        if self.cs_lamp == 0 and self.counter>self.countermax:
            self.counter = 0 #reset the counter
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
        self.idc_lamp() 
        # check whether the command has been complete or not
        # if self.cs_lamp == 1:
        #     self.label_switch_otonom.setText("AUTONOMOUS")
        #     self.switch_netral_2.setText("NEUTRAL")     
        # else: 
            # bring back to stand by mode
        #     self.mode_auto = 0
        #     self.mode_auto_bef = self.mode_auto
        #     self.label_switch_otonom.setText("MANUAL")
        #     self.switch_netral_2.setText("NEUTRAL")
        # publish message
        # self.idc_lamp()        
    
    def mode2(self): #MODE2: Throttle Check
        # check state before
        # print("masuk throttle")
        if self.mode_auto_bef == 0: 
            self.mode_auto = 2
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        self.counter += 1 
        if self.cs_lamp == 0 and self.counter>self.countermax:
            self.counter = 0 #reset the counter
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
        self.idc_lamp() 
        # check whether the command has been complete or not
        # if self.cs_lamp == 1:
        #     self.label_switch_otonom.setText("AUTONOMOUS")            
        # else:
        #     # bring back to stand by mode
        #     self.mode_auto = 0
        #     self.mode_auto_bef = self.mode_auto
        #     self.label_switch_otonom.setText("MANUAL")
        # # publish message
        # self.idc_lamp()

    def mode3(self): #MODE3 = Path following
        # Update 261022 --> temporarily inactive
        self.counter = 0 #reset the counter
        self.mode_auto = 0
        self.mode_auto_bef = self.mode_auto
        self.idc_lamp()
        # check state before
        # print("masuk path")
        # if self.mode_auto_bef == 0: 
        #     self.mode_auto = 3
        #     self.mode_auto_bef = self.mode_auto
        # #     self.cs_lamp = 1
        # self.counter += 1 
        # if self.cs_lamp == 0 and self.counter>self.countermax:
        #     self.counter = 0 #reset the counter
        #     self.mode_auto = 0
        #     self.mode_auto_bef = self.mode_auto
        # self.idc_lamp() 
        # check whether the command has been complete or not
        # if self.cs_lamp == 1:
        #     self.label_switch_otonom.setText("AUTONOMOUS")
        #     self.switch_netral_2.setText("FORWARD")            
        # else:
        #     # bring back to stand by mode
        #     self.mode_auto = 0
        #     self.mode_auto_bef = self.mode_auto
        #     self.label_switch_otonom.setText("MANUAL")
        #     self.switch_netral_2.setText("NEUTRAL")            
        # # publish message
        # self.idc_lamp()
    
    def mode4(self):  #MODE 4 = Point Stabilization        
        # check state before
        # print("masuk point")
        if self.mode_auto_bef == 0: 
            self.mode_auto = 4
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        self.counter += 1 
        if self.cs_lamp == 0 and self.counter>self.countermax:
            self.counter = 0 #reset the counter
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
        self.idc_lamp() 
        # check whether the command has been complete or not
        # if self.cs_lamp == 1:
        #     self.label_switch_otonom.setText("AUTONOMOUS")
        #     self.switch_netral_2.setText("BACKWARD")            
        # else:
        #     # bring back to stand by mode
        #     self.mode_auto = 0
        #     self.mode_auto_bef = self.mode_auto
        #     self.label_switch_otonom.setText("MANUAL")
        #     self.switch_netral_2.setText("NEUTRAL")
        # # publish message
        # self.idc_lamp()  

    def mode5(self):        #MODE 5 = docking
        # check state before
        if self.mode_auto_bef == 0: 
            self.mode_auto = 5
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        self.counter += 1 
        if self.cs_lamp == 0 and self.counter>self.countermax:
            self.counter = 0 #reset the counter
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
        self.idc_lamp() 
        # check whether the command has been complete or not
        # if self.cs_lamp == 1:
        #     self.label_switch_otonom.setText("AUTONOMOUS")
        #     self.switch_netral_2.setText("NEUTRAL")            
        # else:
        #     # bring back to stand by mode
        #     self.mode_auto = 0
        #     self.mode_auto_bef = self.mode_auto
        #     self.label_switch_otonom.setText("MANUAL")
        #     self.switch_netral_2.setText("NEUTRAL")
        # # publish message
        # self.idc_lamp()

    def mode_takeplace(self):   #MODE 10 = take place container
        # check state before
        if self.mode_auto_bef == 0: 
            self.mode_auto = 10
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        self.counter += 1 
        if self.cs_lamp == 0 and self.counter>self.countermax:
            self.counter = 0 #reset the counter
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
        self.idc_lamp() 
        # check whether the command has been complete or not
        # if self.cs_lamp == 1:
        #     self.label_switch_otonom.setText("AUTONOMOUS")
        #     self.switch_netral_2.setText("NEUTRAL")            
        # else:
        #     # bring back to stand by mode
        #     self.mode_auto = 0
        #     self.mode_auto_bef = self.mode_auto
        #     self.label_switch_otonom.setText("MANUAL")
        #     self.switch_netral_2.setText("NEUTRAL")
        # # publish message
        # self.idc_lamp() 

    def mode_undocking(self):   #MODE 9 = undocking
        # check state before
        # print("masuk undoc")
        if self.mode_auto_bef == 0: 
            self.mode_auto = 9
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        self.counter += 1 
        if self.cs_lamp == 0 and self.counter>self.countermax:
            self.counter = 0 #reset the counter
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
        self.idc_lamp() 
        # check whether the command has been complete or not
        # if self.cs_lamp == 1:
        #     self.label_switch_otonom.setText("AUTONOMOUS")
        #     self.switch_netral_2.setText("FORWARD")            
        # else:
        #     # bring back to stand by mode
        #     self.mode_auto = 0
        #     self.mode_auto_bef = self.mode_auto
        #     self.label_switch_otonom.setText("MANUAL")
        #     self.switch_netral_2.setText("NEUTRAL")
        # # publish message
        # self.idc_lamp()

    def mode_liftUp(self):        #MODE5 = liftUp
        # check state before
        if self.mode_auto_bef == 0: 
            self.mode_auto = 6
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        self.counter += 1 
        if self.cs_lamp == 0 and self.counter>self.countermax:
            self.counter = 0 #reset the counter
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
        self.idc_lamp() 
        # check whether the command has been complete or not
        # if self.cs_lamp == 1:
        #     self.label_switch_otonom.setText("AUTONOMOUS")
        #     self.switch_netral_2.setText("NEUTRAL")            
        # else:
        #     # bring back to stand by mode
        #     self.mode_auto = 0
        #     self.mode_auto_bef = self.mode_auto
        #     self.label_switch_otonom.setText("MANUAL")
        #     self.switch_netral_2.setText("NEUTRAL")
        # # publish message
        # self.idc_lamp()
    
    def mode_liftMid(self):        #MODE6 = Lift Mid
        # Update 261022 --> used as cancel/stop button
        self.counter = 0 #reset the counter
        self.mode_auto = 0
        self.mode_auto_bef = self.mode_auto
        self.idc_lamp()
        # check state before
        # if self.mode_auto_bef == 0: 
        #     self.mode_auto = 7
        #     self.mode_auto_bef = self.mode_auto
        # #     self.cs_lamp = 1
        # self.counter += 1 
        # if self.cs_lamp == 0 and self.counter>self.countermax:
        #     self.counter = 0 #reset the counter
        #     self.mode_auto = 0
        #     self.mode_auto_bef = self.mode_auto
        # self.idc_lamp() 
        # self.counter = 0 #reset the counter
        # self.mode_auto = 0
        # self.mode_auto_bef = self.mode_auto
        # self.idc_lamp()
        # check whether the command has been complete or not
        # if self.cs_lamp == 1:
        #     self.label_switch_otonom.setText("AUTONOMOUS")
        #     self.switch_netral_2.setText("NEUTRAL")            
        # else:
        #     # bring back to stand by mode
        #     self.mode_auto = 0
        #     self.mode_auto_bef = self.mode_auto
        #     self.label_switch_otonom.setText("MANUAL")
        #     self.switch_netral_2.setText("NEUTRAL")
        # # publish message
        # self.idc_lamp()
    
    def mode_liftDown(self):        #MODE7 = Lift Down
        # check state before
        if self.mode_auto_bef == 0: 
            self.mode_auto = 8
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        self.counter += 1 
        if self.cs_lamp == 0 and self.counter>self.countermax:
            self.counter = 0 #reset the counter
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
        self.idc_lamp() 
        # check whether the command has been complete or not
        # if self.cs_lamp == 1:
        #     self.label_switch_otonom.setText("AUTONOMOUS")
        #     self.switch_netral_2.setText("NEUTRAL")            
        # else:
        #     # bring back to stand by mode
        #     self.mode_auto = 0
        #     self.mode_auto_bef = self.mode_auto
        #     self.label_switch_otonom.setText("MANUAL")
        #     self.switch_netral_2.setText("NEUTRAL")
        # # publish message
        # self.idc_lamp()

# -------------------------------- Mathplotlib -------------------------------------------------------
    def plot_init(self):
        # self.X_now = 685588.048
        # self.Y_now = 9204433.608
        # self.yaw_now = np.pi/4
        # self.yawrot = np.radians(45.8)
        # self.yaw_head = self.yaw_now

        self.X_now_rot,self.Y_now_rot,self.yaw_now_rot,self.yaw_head_rot = \
                self.rot_coor(self.X_now,self.Y_now,self.yaw_now,self.yaw_head,self.yawrot)

        self._dynamic_ax.set_xlim(-23, 5)
        self._dynamic_ax.set_ylim(-3, 58)
        self._dynamic_ax.set_xlabel("X pos(m)")
        self._dynamic_ax.set_ylabel("Y pos(m)")
        self._dynamic_ax.text(-3.1*5-0.6*5-0.6, 2, f"1")
        self._dynamic_ax.text(0+0.6-1, 2, f"6")
        self._dynamic_ax.text(-3.1*1-0.6-0.6, 2, f"5")
        self._dynamic_ax.text(-3.1*2-0.6*2-0.6, 2, f"4")
        self._dynamic_ax.text(-3.1*3-0.6*3-0.6, 2, f"3")
        self._dynamic_ax.text(-3.1*4-0.6*4-0.6, 2, f"2")
        self._dynamic_ax.text(-14, 51, f"WSTA BLOK 1")

        self.ln, = self._dynamic_ax.plot([], [], ".", label="CTT Ref Point")
        self.cfw, = self._dynamic_ax.plot([], [], ".") 
        self.bound, = self._dynamic_ax.plot([], [], "k")
        self.trailer_plot, = self._dynamic_ax.plot([], [], "g")
        self.head_plot, = self._dynamic_ax.plot([], [], "g")
        self.wheel1_plot, = self._dynamic_ax.plot([], [], "b")
        self.wheel2_plot, = self._dynamic_ax.plot([], [], "b")
        self.wheel3_plot, = self._dynamic_ax.plot([], [], "b")
        self.wheel4_plot, = self._dynamic_ax.plot([], [], "b")
        self.l_dock, = self._dynamic_ax.plot([], [], "m")
        self.r_dock, = self._dynamic_ax.plot([], [], "m")
        self.b_dock, = self._dynamic_ax.plot([], [], "m")
        self.l_dock1, = self._dynamic_ax.plot([], [], "m")
        self.r_dock1, = self._dynamic_ax.plot([], [], "m")
        self.b_dock1, = self._dynamic_ax.plot([], [], "m")
        self.l_dock3, = self._dynamic_ax.plot([], [], "m")
        self.r_dock3, = self._dynamic_ax.plot([], [], "m")
        self.b_dock3, = self._dynamic_ax.plot([], [], "m")
        self.l_dock4, = self._dynamic_ax.plot([], [], "m")
        self.r_dock4, = self._dynamic_ax.plot([], [], "m")
        self.b_dock4, = self._dynamic_ax.plot([], [], "m")
        self.l_dock5, = self._dynamic_ax.plot([], [], "m")
        self.r_dock5, = self._dynamic_ax.plot([], [], "m")
        self.b_dock5, = self._dynamic_ax.plot([], [], "m")
        self.l_dock6, = self._dynamic_ax.plot([], [], "m")
        self.r_dock6, = self._dynamic_ax.plot([], [], "m")
        self.b_dock6, = self._dynamic_ax.plot([], [], "m")
        self.target_PSback, = self._dynamic_ax.plot([],[], ".")
        self.target_PSdock, = self._dynamic_ax.plot([],[], ".")
        self._dynamic_ax.legend(loc=2, prop={'size': 6})


        # self.control_mode = 0 --> self.mode_auto
    
    def plot_edge_car(self,x,y,yaw,rot_x,rot_y):

        x1 = x + 6.7*rot_x*np.cos(yaw) - rot_y*np.sin(yaw)
        y1 = y + 6.7*rot_x*np.sin(yaw) + rot_y*np.cos(yaw)

        x2 = x - rot_x*np.cos(yaw) - rot_y*np.sin(yaw)
        y2 = y - rot_x*np.sin(yaw) + rot_y*np.cos(yaw)

        x3 = x - rot_x*np.cos(yaw) + rot_y*np.sin(yaw)
        y3 = y - rot_x*np.sin(yaw) - rot_y*np.cos(yaw)

        x4 = x + 6.7*rot_x*np.cos(yaw) + rot_y*np.sin(yaw)
        y4 = y + 6.7*rot_x*np.sin(yaw) - rot_y*np.cos(yaw)

        xy = [[x1,y1],[x2,y2],[x3,y3],[x4,y4],[x1,y1]]

        return xy

    def plot_edge_wheel(self, x,y,yaw,rot_x,rot_y):
        
        x1 = x + rot_x*np.cos(yaw) - rot_y*np.sin(yaw)
        y1 = y + rot_x*np.sin(yaw) + rot_y*np.cos(yaw)

        x2 = x - rot_x*np.cos(yaw) - rot_y*np.sin(yaw)
        y2 = y - rot_x*np.sin(yaw) + rot_y*np.cos(yaw)

        x3 = x - rot_x*np.cos(yaw) + rot_y*np.sin(yaw)
        y3 = y - rot_x*np.sin(yaw) - rot_y*np.cos(yaw)

        x4 = x + rot_x*np.cos(yaw) + rot_y*np.sin(yaw)
        y4 = y + rot_x*np.sin(yaw) - rot_y*np.cos(yaw)

        xy = [[x1,y1],[x2,y2],[x3,y3],[x4,y4],[x1,y1]]

        return xy
    
    def trailer_(self): #(x position, y position, yaw angle, length, width)
        # Create polygon coordinates
        plot_t_x = 1.8
        plot_t_y = 1
        
        edge_trailer = self.plot_edge_car(self.X_now_rot, self.Y_now_rot, self.yaw_now_rot, plot_t_x, plot_t_y)

        return edge_trailer  

    def head_(self): 
        # create head center point
        plot_h_x = 0.45
        plot_h_y = 1

        edge_head = self.plot_edge_car(self.X_h_now, self.Y_h_now, self.yaw_head_rot, plot_h_x, plot_h_y)

        return edge_head                
    
    def wheel1_(self): 
        # create head center point

        edge_wheel_1 = self.plot_edge_wheel(self.X_h_now - 1*np.sin(self.yaw_head_rot), self.Y_h_now + 1*np.cos(self.yaw_head_rot), self.yaw_head_rot, 0.6, 0.3)

        return edge_wheel_1 

    def wheel2_(self): 
        # create head center point

        edge_wheel_2 = self.plot_edge_wheel(self.X_h_now + 1*np.sin(self.yaw_head_rot), self.Y_h_now - 1*np.cos(self.yaw_head_rot), self.yaw_head_rot, 0.6, 0.3)

        return edge_wheel_2

    def wheel3_(self): 
        # create head center point

        edge_wheel_3 = self.plot_edge_wheel(self.X_now_rot - 1*np.sin(self.yaw_now_rot), self.Y_now_rot + 1*np.cos(self.yaw_now_rot), self.yaw_now_rot, 0.6, 0.3)

        return edge_wheel_3 

    def wheel4_(self): 
        # create head center point

        edge_wheel_4 = self.plot_edge_wheel(self.X_now_rot + 1*np.sin(self.yaw_now_rot), self.Y_now_rot - 1*np.cos(self.yaw_now_rot), self.yaw_now_rot, 0.6, 0.3)

        return edge_wheel_4 

    def dock_(self,dock):

        x_coor = [dock[0],dock[1]]
        y_coor = [dock[2],dock[3]]

        return x_coor, y_coor
    
    def rot_coor(self,x,y,yaw,yawhead,yawrot):
        # xcr = (x-self.PSdockX) * np.cos(yawrot)-(y-self.PSdockY) * np.sin(yawrot)
        # ycr = (x-self.PSdockX) * np.sin(yawrot)+(y-self.PSdockY) * np.cos(yawrot)
        # tetacr = yaw + yawrot
        # tetacrr = yawhead + yawrot
        xcr = (x-self.PSdockX) * np.cos(yawrot)+(y-self.PSdockY) * np.sin(yawrot)
        ycr = -(x-self.PSdockX) * np.sin(yawrot)+(y-self.PSdockY) * np.cos(yawrot)
        tetacr = yaw - yawrot
        tetacrr = yawhead - yawrot

        return xcr,ycr,tetacr,tetacrr
    
#     def sub_data_conv(self,msg):
#         self.X_now = msg.pose.covariance[4]
#         self.Y_now = msg.pose.covariance[5]
#         self.yaw_now = msg.pose.covariance[7]
#         self.yaw_head = msg.pose.covariance[8]

#     def callback_HMI(self,msg):
#         self.control_mode = msg.pose.covariance[0]
 
    def _update_canvas(self):
        PSbackX_rot,PSbackY_rot,PSbackyaw_rot,PSt = \
                self.rot_coor(self.PSbackX,self.PSbackY,self.yawrot, self.yawrot, self.yawrot)
        PSdockX_rot,PSdockY_rot,PSdockyaw_rot,PSh = \
                self.rot_coor(self.PSdockX,self.PSdockY,self.yawrot, self.yawrot, self.yawrot)

        if self.mode_auto == 4: 
            self.target_PSback.set_data(PSbackX_rot, PSbackY_rot)
            self.target_PSback.figure.canvas.draw()
        elif self.mode_auto == 5:
            self.target_PSdock.set_data(PSdockX_rot, PSdockY_rot)
            self.target_PSdock.figure.canvas.draw()
        else: 
            self._dynamic_ax.cla()
            self.plot_init()

        self.X_now_rot,self.Y_now_rot,self.yaw_now_rot,self.yaw_head_rot = \
                self.rot_coor(self.X_now,self.Y_now,self.yaw_now,self.yaw_head,self.yawrot)

        self.X_h_now = self.X_now_rot + 11.29*np.cos(self.yaw_now_rot)
        self.Y_h_now = self.Y_now_rot + 11.29*np.sin(self.yaw_now_rot)

        self.ln.set_data(self.X_now_rot, self.Y_now_rot)
        self.ln.figure.canvas.draw()

        self.bound.set_data([-22,4,4,-22,-22], [-2,-2,55,55,-2])
        self.bound.figure.canvas.draw()

        self.cfw.set_data(self.X_h_now, self.Y_h_now)
        self.cfw.figure.canvas.draw()

        AV_trailer = self.trailer_()
        x_t,y_t = Polygon(AV_trailer).exterior.xy
        self.trailer_plot.set_data(x_t,y_t)
        self.trailer_plot.figure.canvas.draw()

        AV_head = self.head_()
        x_h,y_h = Polygon(AV_head).exterior.xy
        self.head_plot.set_data(x_h,y_h)
        self.head_plot.figure.canvas.draw()

        AV_wheel1 = self.wheel1_()
        x_w_1,y_w_1 = Polygon(AV_wheel1).exterior.xy
        self.wheel1_plot.set_data(x_w_1,y_w_1)
        self.wheel1_plot.figure.canvas.draw()

        AV_wheel2 = self.wheel2_()
        x_w_2,y_w_2 = Polygon(AV_wheel2).exterior.xy
        self.wheel2_plot.set_data(x_w_2,y_w_2)
        self.wheel2_plot.figure.canvas.draw()

        AV_wheel3 = self.wheel3_()
        x_w_3,y_w_3 = Polygon(AV_wheel3).exterior.xy
        self.wheel3_plot.set_data(x_w_3,y_w_3)
        self.wheel3_plot.figure.canvas.draw()

        AV_wheel4 = self.wheel4_()
        x_w_4,y_w_4 = Polygon(AV_wheel4).exterior.xy
        self.wheel4_plot.set_data(x_w_4,y_w_4)
        self.wheel4_plot.figure.canvas.draw()

        # create docking station
        # x_l_d, y_l_d = self.dock_(self.l_dock_)
        x_l_d = [self.l_dock_[0],self.l_dock_[2]]
        y_l_d = [self.l_dock_[1],self.l_dock_[3]]
        self.l_dock.set_data(x_l_d, y_l_d)
        self.l_dock.figure.canvas.draw()

        # x_r_d, y_r_d = self.dock_(self.r_dock_)
        x_r_d = [self.r_dock_[0],self.r_dock_[2]]
        y_r_d = [self.r_dock_[1],self.r_dock_[3]]
        self.r_dock.set_data(x_r_d, y_r_d)
        self.r_dock.figure.canvas.draw()

        # x_b_d, y_b_d = self.dock_(self.b_dock_)
        x_b_d = [self.b_dock_[0],self.b_dock_[2]]
        y_b_d = [self.b_dock_[1],self.b_dock_[3]]
        self.b_dock.set_data(x_b_d, y_b_d)
        self.b_dock.figure.canvas.draw()

        x_l_d1 = [self.l_dock_1[0],self.l_dock_1[2]]
        y_l_d1 = [self.l_dock_1[1],self.l_dock_1[3]]
        self.l_dock1.set_data(x_l_d1, y_l_d1)
        self.l_dock1.figure.canvas.draw()

        # x_r_d, y_r_d = self.dock_(self.r_dock_)
        x_r_d1 = [self.r_dock_1[0],self.r_dock_1[2]]
        y_r_d1 = [self.r_dock_1[1],self.r_dock_1[3]]
        self.r_dock1.set_data(x_r_d1, y_r_d1)
        self.r_dock1.figure.canvas.draw()

        # x_b_d, y_b_d = self.dock_(self.b_dock_)
        x_b_d1 = [self.b_dock_1[0],self.b_dock_1[2]]
        y_b_d1 = [self.b_dock_1[1],self.b_dock_1[3]]
        self.b_dock1.set_data(x_b_d1, y_b_d1)
        self.b_dock1.figure.canvas.draw()

        x_l_d3 = [self.l_dock_3[0],self.l_dock_3[2]]
        y_l_d3 = [self.l_dock_3[1],self.l_dock_3[3]]
        self.l_dock3.set_data(x_l_d3, y_l_d3)
        self.l_dock3.figure.canvas.draw()

        # x_r_d, y_r_d = self.dock_(self.r_dock_)
        x_r_d3 = [self.r_dock_3[0],self.r_dock_3[2]]
        y_r_d3 = [self.r_dock_3[1],self.r_dock_3[3]]
        self.r_dock3.set_data(x_r_d3, y_r_d3)
        self.r_dock3.figure.canvas.draw()

        # x_b_d, y_b_d = self.dock_(self.b_dock_)
        x_b_d3 = [self.b_dock_3[0],self.b_dock_3[2]]
        y_b_d3 = [self.b_dock_3[1],self.b_dock_3[3]]
        self.b_dock3.set_data(x_b_d3, y_b_d3)
        self.b_dock3.figure.canvas.draw()

        x_l_d4 = [self.l_dock_4[0],self.l_dock_4[2]]
        y_l_d4 = [self.l_dock_4[1],self.l_dock_4[3]]
        self.l_dock4.set_data(x_l_d4, y_l_d4)
        self.l_dock4.figure.canvas.draw()

        # x_r_d, y_r_d = self.dock_(self.r_dock_)
        x_r_d4 = [self.r_dock_4[0],self.r_dock_4[2]]
        y_r_d4 = [self.r_dock_4[1],self.r_dock_4[3]]
        self.r_dock4.set_data(x_r_d4, y_r_d4)
        self.r_dock4.figure.canvas.draw()

        # x_b_d, y_b_d = self.dock_(self.b_dock_)
        x_b_d4 = [self.b_dock_4[0],self.b_dock_4[2]]
        y_b_d4 = [self.b_dock_4[1],self.b_dock_4[3]]
        self.b_dock4.set_data(x_b_d4, y_b_d4)
        self.b_dock4.figure.canvas.draw()

        x_l_d5 = [self.l_dock_5[0],self.l_dock_5[2]]
        y_l_d5 = [self.l_dock_5[1],self.l_dock_5[3]]
        self.l_dock5.set_data(x_l_d5, y_l_d5)
        self.l_dock5.figure.canvas.draw()

        # x_r_d, y_r_d = self.dock_(self.r_dock_)
        x_r_d5 = [self.r_dock_5[0],self.r_dock_5[2]]
        y_r_d5 = [self.r_dock_5[1],self.r_dock_5[3]]
        self.r_dock5.set_data(x_r_d5, y_r_d5)
        self.r_dock5.figure.canvas.draw()

        # x_b_d, y_b_d = self.dock_(self.b_dock_)
        x_b_d5 = [self.b_dock_5[0],self.b_dock_5[2]]
        y_b_d5 = [self.b_dock_5[1],self.b_dock_5[3]]
        self.b_dock5.set_data(x_b_d5, y_b_d5)
        self.b_dock5.figure.canvas.draw()

        x_l_d6 = [self.l_dock_6[0],self.l_dock_6[2]]
        y_l_d6 = [self.l_dock_6[1],self.l_dock_6[3]]
        self.l_dock6.set_data(x_l_d6, y_l_d6)
        self.l_dock6.figure.canvas.draw()

        # x_r_d, y_r_d = self.dock_(self.r_dock_)
        x_r_d6 = [self.r_dock_6[0],self.r_dock_6[2]]
        y_r_d6 = [self.r_dock_6[1],self.r_dock_6[3]]
        self.r_dock6.set_data(x_r_d6, y_r_d6)
        self.r_dock6.figure.canvas.draw()

        # x_b_d, y_b_d = self.dock_(self.b_dock_)
        x_b_d6 = [self.b_dock_6[0],self.b_dock_6[2]]
        y_b_d6 = [self.b_dock_6[1],self.b_dock_6[3]]
        self.b_dock6.set_data(x_b_d6, y_b_d6)
        self.b_dock6.figure.canvas.draw()

        # print("printing")
        # print(self.counter)
        # print(self.mode_auto)
        # print("end")
   
#     def plot_init(self):
#         # plot init
#         self._dynamic_ax.set_xlim(-7, -7)
#         self._dynamic_ax.set_ylim(-3, 50)
#         self._dynamic_ax.set_xlabel("X_pos(m)")
#         self._dynamic_ax.set_ylabel("Y_pos(m)")
#         # self._dynamic_ax.axis("equal")
#         # Set up plot variable
#         self._pos, = self._dynamic_ax.plot([],[],'.',label="AGV position")
#         self._bound, = self._dynamic_ax.plot([], [], "k")
#         self._trailer_plot, = self._dynamic_ax.plot([], [], "g")
#         self._head_plot, = self._dynamic_ax.plot([], [], "g")
#         self._waypoints, = self._dynamic_ax.plot([], [], "r")
#         self.l_dock_, = self._dynamic_ax.plot([], [], "m")
#         self.r_dock_, = self._dynamic_ax.plot([], [], "m")
#         self.low_dock_, = self._dynamic_ax.plot([], [], "m")
#         self._target_PS, = self._dynamic_ax.plot([], [], ".")
#         self._target_dock, = self._dynamic_ax.plot([], [], ".")
#         self._waypoint_path_zero, = self._dynamic_ax.plot([], [], ".", label="start path follow")
#         # self._waypoint_path_zero, = self._dynamic_ax.plot([], [], ".", label="start ps")
#         self.xtodock, self.ytodock = 558098.5399999999, 9158001.32#min(self.X_utm) + 14.5, min(self.Y_utm) + 70
#         self.start_to_dock, = plt.plot([],[], ".", label="start point")
#         self.cfw, = self._dynamic_ax.plot([], [], ".")
#         self.start_path, = self._dynamic_ax.plot([], [], ".")
#         self._dynamic_ax.legend(loc=2, prop={'size': 6})

#     def _update_canvas(self):
#         if self.mode_auto == 3: 
#             self._waypoints.set_data(self.waypoints[:,0],self.waypoints[:,1])
#             self._waypoints.figure.canvas.draw()
#         elif self.mode_auto == 4: 
#             self._target_PS.set_data(self.PSbackX, self.PSbackY)
#             self._target_PS.figure.canvas.draw()
#         elif self.mode_auto == 5:
#             self._target_dock.set_data(self.PSdockX, self.PSdockY)
#             self._target_dock.figure.canvas.draw()
#         else: 
#             # clear the plotting area
#             self._dynamic_ax.cla()
#             self.plot_init()
        
#         self.cfw.set_data(self.X_now + 11.92 * np.cos(self.yaw_now), \
#                             self.Y_now + 11.92 * np.sin(self.yaw_now))
#         self.start_path.set_data(self.waypoints[:,0][0],self.waypoints[:,1][1])
#         # set data_update
#         self._pos.set_data(self.X_now, self.Y_now)
#         self._bound.set_data(self.X_utm, self.Y_utm)
#         # AV_Trailer = self.Trailer_(self.X_now, self.Y_now, self.yaw_now)
#         AV_trailer = self.Trailer_(self.X_now + 11.291/2 * np.cos(self.yaw_now)\
#             , self.Y_now + 11.291/2 * np.sin(self.yaw_now), self.yaw_now)
#         x3,y3 = Polygon(AV_trailer).exterior.xy
#         self._trailer_plot.set_data(x3,y3)
#         # AV_Head = self.head_(self.X_now, self.Y_now, self.yaw_now, self.yaw_head)
#         AV_head = self.head_(self.X_now + 11.291/2 * np.cos(self.yaw_now)\
#             , self.Y_now + 11.291/2 * np.sin(self.yaw_now), self.yaw_now, self.yaw_head)
#         x4,y4 = Polygon(AV_head).exterior.xy
#         self._head_plot.set_data(x4,y4)
#         # print(x4,y4)
#         wpx = self.waypoints[:,0]
#         wpy = self.waypoints[:,1]+11.92
#         # self._waypoint_path_zero.set_data(wpx[0],wpy[0])
#         self._waypoint_path_zero.set_data(self.xtodock, self.ytodock)
#         # create docking station
#         l_dock_ = self.l_dock_pos
#         r_dock_ = self.r_dock_pos
#         low_dock_ = self.low_dock_pos
#         l_dock_box = self.dock_(l_dock_[0],l_dock_[1],l_dock_[2],l_dock_[3],l_dock_[4])
#         x5,y5 = Polygon(l_dock_box).exterior.xy
#         # print(x5,y5)
#         self.l_dock_.set_data(x5,y5)
#         r_dock_box = self.dock_(r_dock_[0],r_dock_[1],r_dock_[2],r_dock_[3],r_dock_[4])
#         x6,y6 = Polygon(r_dock_box).exterior.xy
#         self.r_dock_.set_data(x6,y6)
#         low_dock_box = self.dock_(low_dock_[0],low_dock_[1],low_dock_[2],low_dock_[3],low_dock_[4])
#         x7,y7 = Polygon(low_dock_box).exterior.xy
#         self.low_dock_.set_data(x7,y7)

#         self.start_to_dock.set_data(self.xtodock, self.ytodock)

#         # draw the data visualization
#         self.start_to_dock.figure.canvas.draw()
#         self._pos.figure.canvas.draw()
#         self._trailer_plot.figure.canvas.draw()
#         self._head_plot.figure.canvas.draw()
#         self._bound.figure.canvas.draw()
#         self._waypoint_path_zero.figure.canvas.draw()
#         self.l_dock_.figure.canvas.draw()
#         self.r_dock_.figure.canvas.draw()
#         self.low_dock_.figure.canvas.draw()
#         # print("hereeeee")
#         plt.axis("equal")
#         print("printing")
#         print(self.counter)
#         print(self.mode_auto)
#         print("end")
#         # self._dynamic_ax.legend()

#     def dock_(self, X_pos, Y_pos, yaw_,l_, w_): #(x position, y position, yaw angle, length, width)
#         # Create polygon coordinates
#         transform_coor = [[X_pos-l_/2*np.cos(yaw_)+w_/2*np.sin(yaw_),Y_pos-l_/2*np.sin(yaw_)-w_/2*np.cos(yaw_)],
#                 [X_pos-l_/2*np.cos(yaw_)-w_/2*np.sin(yaw_),Y_pos-l_/2*np.sin(yaw_)+w_/2*np.cos(yaw_)],
#                 [X_pos+l_/2*np.cos(yaw_)-w_/2*np.sin(yaw_),Y_pos+l_/2*np.sin(yaw_)+w_/2*np.cos(yaw_)],
#                 [X_pos+l_/2*np.cos(yaw_)+w_/2*np.sin(yaw_),Y_pos+l_/2*np.sin(yaw_)-w_/2*np.cos(yaw_)]]
#         return transform_coor
    
#     def Trailer_(self, X_pos, Y_pos, yaw_): #(x position, y position, yaw angle, length, width)
#         # Create polygon coordinates
#         transform_coor = [[X_pos-self.l_t/2*np.cos(yaw_)+self.w_t/2*np.sin(yaw_),Y_pos-self.l_t/2*np.sin(yaw_)-self.w_t/2*np.cos(yaw_)],
#                 [X_pos-self.l_t/2*np.cos(yaw_)-self.w_t/2*np.sin(yaw_),Y_pos-self.l_t/2*np.sin(yaw_)+self.w_t/2*np.cos(yaw_)],
#                 [X_pos+self.l_t/2*np.cos(yaw_)-self.w_t/2*np.sin(yaw_),Y_pos+self.l_t/2*np.sin(yaw_)+self.w_t/2*np.cos(yaw_)],
#                 [X_pos+self.l_t/2*np.cos(yaw_)+self.w_t/2*np.sin(yaw_),Y_pos+self.l_t/2*np.sin(yaw_)-self.w_t/2*np.cos(yaw_)]]
#         return transform_coor  

#     def head_(self, X_pos_t, Y_pos_t, yaw_t, yaw_h): 
#         # create head center point
#         h_center = [X_pos_t + (self.l_t/2-0.755)*np.cos(yaw_t), Y_pos_t + (self.l_t/2-0.755)*np.sin(yaw_t)]
#         h_transform = [[h_center[0]-self.l_h/2*np.cos(yaw_h)+self.w_h/2*np.sin(yaw_h),h_center[1]-self.l_h/2*np.sin(yaw_h)-self.w_h/2*np.cos(yaw_h)],
#                 [h_center[0]-self.l_h/2*np.cos(yaw_h)-self.w_h/2*np.sin(yaw_h),h_center[1]-self.l_h/2*np.sin(yaw_h)+self.w_h/2*np.cos(yaw_h)],
#                 [h_center[0]+self.l_h/2*np.cos(yaw_h)-self.w_h/2*np.sin(yaw_h),h_center[1]+self.l_h/2*np.sin(yaw_h)+self.w_h/2*np.cos(yaw_h)],
#                 [h_center[0]+self.l_h/2*np.cos(yaw_h)+self.w_h/2*np.sin(yaw_h),h_center[1]+self.l_h/2*np.sin(yaw_h)-self.w_h/2*np.cos(yaw_h)]]
#         return h_transform

if __name__ == "__main__":
#     import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

