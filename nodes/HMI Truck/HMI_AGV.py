#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_main.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

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

# IMPORT FUNCTIONS
from ui_functions import *

# docking spec (x,y,orientation,length,width)
# left docking
l_dock_ = [558088.9866405, 9157947.32, 0.5*np.pi, 10, 0.2]
# right docking
r_dock_ = [558094.9866405,  9157947.32, 0.5*np.pi, 10, 0.2]
# lower docking
low_dock_ = [558091.9866405, 9157942.32, np.pi, 6, 0.2]
# print("HIAAAHAHAHAHAHAHAHAHA")


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):  
        # Subscribe ROS
        rospy.init_node('node_hmi')
        self.sub = rospy.Subscriber('/data_hmi', data_collector, self.callback_data_collector)
        
        # Publish ROS
        self.pub = rospy.Publisher('/idc_lamp', PoseWithCovarianceStamped, queue_size=10)
        self.msg = PoseWithCovarianceStamped()

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
        self.cs_lamp = 1
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
        self.X_now = 558099.34
        self.Y_now = 9157952.32
        self.yaw_now = 0.5*np.pi
        
        # waypoint information
        waypoints_path = rospy.get_param('~waypoints_path', 'sigmoid160522.npy')
        waypoints_path = os.path.abspath(sys.path[0] + '/../../src/waypoints/' + waypoints_path)
        self.PSbackX = rospy.get_param('~PSback_X', 558091.866)#558088.34)
        self.PSbackY = rospy.get_param('~PSback_Y', 9157947.658)#9157952.32)
        self.PSdockX = rospy.get_param('~dock_X', 558091.816)#558088.34)
        self.PSdockY = rospy.get_param('~dock_Y', 9157928.938)#9157941.199000001)
        self.waypoints = np.load(waypoints_path)
        self.yaw_head = 0.5*np.pi
        # docking spec (x,y,orientation,length,width)
        # left docking
        self.l_dock_pos = [558088.816, 9157931.938, 0.5*np.pi, 10, 0.2]
        # right docking
        self.r_dock_pos = [558094.816,  9157931.938, 0.5*np.pi, 10, 0.2]
        # lower docking
        self.low_dock_pos = [558091.816, 9157926.938, np.pi, 6, 0.2]
        
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
        self.Top_Bar = QtWidgets.QFrame(self.centralwidget)
        self.Top_Bar.setMaximumSize(QtCore.QSize(16777215, 40))
        self.Top_Bar.setStyleSheet("background-color: rgb(35, 35, 35);")
        self.Top_Bar.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.Top_Bar.setFrameShadow(QtWidgets.QFrame.Raised)
        self.Top_Bar.setObjectName("Top_Bar")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.Top_Bar)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.frame_toggle = QtWidgets.QFrame(self.Top_Bar)
        self.frame_toggle.setMaximumSize(QtCore.QSize(70, 40))
        self.frame_toggle.setStyleSheet("background-color: rgb(85, 170, 255);")
        self.frame_toggle.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_toggle.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_toggle.setObjectName("frame_toggle")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.frame_toggle)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.Btn_menu = QtWidgets.QPushButton(self.frame_toggle)
        self.Btn_menu.clicked.connect(lambda: UIFunctions.toggleMenu(self, 250, True))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Btn_menu.sizePolicy().hasHeightForWidth())
        self.Btn_menu.setSizePolicy(sizePolicy)
        self.Btn_menu.setStyleSheet("color: rgb(255, 255, 255);\n"
"border: 0px solid;")
        self.Btn_menu.setObjectName("Btn_menu")
        self.verticalLayout_2.addWidget(self.Btn_menu)
        self.horizontalLayout.addWidget(self.frame_toggle)
        self.frame_top = QtWidgets.QFrame(self.Top_Bar)
        self.frame_top.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_top.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_top.setObjectName("frame_top")
        self.label_title = QtWidgets.QLabel(self.frame_top)
        self.label_title.setGeometry(QtCore.QRect(350, 10, 401, 17))
        self.label_title.setObjectName("label_title")
        self.horizontalLayout.addWidget(self.frame_top)
        self.verticalLayout.addWidget(self.Top_Bar)
        self.Content = QtWidgets.QFrame(self.centralwidget)
        self.Content.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.Content.setFrameShadow(QtWidgets.QFrame.Raised)
        self.Content.setObjectName("Content")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.Content)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.frame_left_menu = QtWidgets.QFrame(self.Content)
        self.frame_left_menu.setEnabled(True)
        self.frame_left_menu.setMinimumSize(QtCore.QSize(70, 0))
        self.frame_left_menu.setMaximumSize(QtCore.QSize(70, 16777215))
        self.frame_left_menu.setStyleSheet("background-color: rgb(35, 35, 35);")
        self.frame_left_menu.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_left_menu.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_left_menu.setObjectName("frame_left_menu")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.frame_left_menu)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.frame_top_menus = QtWidgets.QFrame(self.frame_left_menu)
        self.frame_top_menus.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.frame_top_menus.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_top_menus.setObjectName("frame_top_menus")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.frame_top_menus)
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_4.setSpacing(0)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        
        self.btn_page_1 = QtWidgets.QPushButton(self.frame_top_menus)
        self.btn_page_1.setMinimumSize(QtCore.QSize(0, 40))
        self.btn_page_1.setStyleSheet("QPushButton {\n"
"    color: rgb(255, 255, 255);\n"
"    background-color: rgb(35, 35, 35);\n"
"    border: 0px solid;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(85, 170, 255);\n"
"}")
        self.btn_page_1.setObjectName("btn_page_1")
        self.btn_page_1.clicked.connect(lambda: self.stackedWidget.setCurrentWidget(self.page_1_dashboard))

        self.verticalLayout_4.addWidget(self.btn_page_1)

        self.btn_page_2 = QtWidgets.QPushButton(self.frame_top_menus)
        self.btn_page_2.setMinimumSize(QtCore.QSize(0, 40))
        self.btn_page_2.setStyleSheet("QPushButton {\n"
"    color: rgb(255, 255, 255);\n"
"    background-color: rgb(35, 35, 35);\n"
"    border: 0px solid;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(85, 170, 255);\n"
"}")
        self.btn_page_2.setObjectName("btn_page_2")
        self.btn_page_2.clicked.connect(lambda: self.stackedWidget.setCurrentWidget(self.page_2_log))
        self.verticalLayout_4.addWidget(self.btn_page_2)
        self.verticalLayout_3.addWidget(self.frame_top_menus, 0, QtCore.Qt.AlignTop)
        self.horizontalLayout_2.addWidget(self.frame_left_menu)
        self.frame_pages = QtWidgets.QFrame(self.Content)
        self.frame_pages.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_pages.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_pages.setObjectName("frame_pages")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.frame_pages)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.stackedWidget = QtWidgets.QStackedWidget(self.frame_pages)
        self.stackedWidget.setObjectName("stackedWidget")
        
# --------------------------------------- Page 1 -------------------------------------        
        self.page_1_dashboard = QtWidgets.QWidget()
        self.page_1_dashboard.setObjectName("page_1_dashboard")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.page_1_dashboard)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.frame = QtWidgets.QFrame(self.page_1_dashboard)
        self.frame.setStyleSheet("background-color: rgb(77, 77, 127);")
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        
#----------------------------------- Button Autonomous Check -------------------------
        self.button_path = QtWidgets.QPushButton(self.frame)
        self.button_path.setGeometry(QtCore.QRect(850, 580, 91, 51))
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
        
        self.button_ps = QtWidgets.QPushButton(self.frame)
        self.button_ps.setGeometry(QtCore.QRect(960, 580, 91, 51))
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
        
        self.button_dock = QtWidgets.QPushButton(self.frame)
        self.button_dock.setGeometry(QtCore.QRect(1070, 580, 91, 51))
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

        self.button_check_steer = QtWidgets.QPushButton(self.frame)
        self.button_check_steer.setGeometry(QtCore.QRect(850, 10, 151, 51))
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
        
        self.button_check_throttle = QtWidgets.QPushButton(self.frame)
        self.button_check_throttle.setGeometry(QtCore.QRect(1020, 10, 141, 51))
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

# ------------------------------- Image CTT ---------------------------        
        self.imageCTT = QtWidgets.QLabel(self.frame)
        self.imageCTT.setGeometry(QtCore.QRect(351, 88, 101, 381))
        self.imageCTT.setAutoFillBackground(False)
        self.imageCTT.setText("")
        self.imageCTT.setScaledContents(True)
        self.imageCTT.setObjectName("imageCTT")
        self.qpixmap = QPixmap('/home/agv-ctt/catkin_CTT/src/AGV_CTT/nodes/HMI Truck/truck.png')
        self.imageCTT.setPixmap(self.qpixmap)
        self.imageCTT.resize(100,375)

# ------------------------------- Speed in Dashboard --------------------

        self.circular_feedSpeed = QtWidgets.QFrame(self.frame)
        self.circular_feedSpeed.setGeometry(QtCore.QRect(540, 180, 240, 240))
        self.circular_feedSpeed.setStyleSheet("background-color: none;")
        self.circular_feedSpeed.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.circular_feedSpeed.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circular_feedSpeed.setObjectName("circular_feedSpeed")
        self.circularBg_feedSpeed_2 = QtWidgets.QFrame(self.circular_feedSpeed)
        self.circularBg_feedSpeed_2.setGeometry(QtCore.QRect(10, 10, 220, 220))
        self.circularBg_feedSpeed_2.setStyleSheet("QFrame{\n"
"    border-radius: 110px;    \n"
"    background-color: rgba(85, 85, 127, 100);\n"
"}")
        self.circularBg_feedSpeed_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.circularBg_feedSpeed_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circularBg_feedSpeed_2.setObjectName("circularBg_feedSpeed_2")
        self.animasi_feedSpeed_2 = QtWidgets.QFrame(self.circularBg_feedSpeed_2)
        self.animasi_feedSpeed_2.setGeometry(QtCore.QRect(0, 0, 220, 220))
        self.animasi_feedSpeed_2.setStyleSheet("QFrame{\n"
"    border-radius: 110px;    \n"
"    background-color: qconicalgradient(cx:0.5, cy:0.5, angle:90, stop:0.750 rgba(255, 0, 127, 255), stop:0.745 rgba(255, 255, 255, 0));\n"
"}")
        self.animasi_feedSpeed_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.animasi_feedSpeed_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.animasi_feedSpeed_2.setObjectName("animasi_feedSpeed_2")
        self.animasi()
        self.circularContainer_feedSpeed_2 = QtWidgets.QFrame(self.circular_feedSpeed)
        self.circularContainer_feedSpeed_2.setGeometry(QtCore.QRect(25, 25, 190, 190))
        self.circularContainer_feedSpeed_2.setBaseSize(QtCore.QSize(0, 0))
        self.circularContainer_feedSpeed_2.setStyleSheet("QFrame{\n"
"    border-radius: 95px;    \n"
"    background-color: rgb(58, 58, 102);\n"
"}")
        self.circularContainer_feedSpeed_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.circularContainer_feedSpeed_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circularContainer_feedSpeed_2.setObjectName("circularContainer_feedSpeed_2")
        self.layoutWidget_4 = QtWidgets.QWidget(self.circularContainer_feedSpeed_2)
        self.layoutWidget_4.setGeometry(QtCore.QRect(10, 40, 171, 133))
        self.layoutWidget_4.setObjectName("layoutWidget_4")
        self.infoLayout_feedSpeed_2 = QtWidgets.QGridLayout(self.layoutWidget_4)
        self.infoLayout_feedSpeed_2.setContentsMargins(0, 0, 0, 0)
        self.infoLayout_feedSpeed_2.setObjectName("infoLayout_feedSpeed_2")
        self.label_feedSpeed_2 = QtWidgets.QLabel(self.layoutWidget_4)
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(10)
        self.label_feedSpeed_2.setFont(font)
        self.label_feedSpeed_2.setStyleSheet("color: #FFFFFF; background-color: none;")
        self.label_feedSpeed_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_feedSpeed_2.setObjectName("label_feedSpeed_2")
        self.infoLayout_feedSpeed_2.addWidget(self.label_feedSpeed_2, 0, 0, 1, 1)
        self.label_progress_commandSpeed_3 = QtWidgets.QLabel(self.layoutWidget_4)
        font = QtGui.QFont()
        font.setFamily("Roboto Thin")
        font.setPointSize(30)
        # Angka yang di rubah
        self.label_progress_commandSpeed_3.setFont(font)
        self.label_progress_commandSpeed_3.setStyleSheet("color: rgb(255, 44, 174); padding: 0px; background-color: none;")
        self.label_progress_commandSpeed_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_progress_commandSpeed_3.setIndent(-1)
        self.label_progress_commandSpeed_3.setObjectName("label_progress_commandSpeed_3")
        
        self.infoLayout_feedSpeed_2.addWidget(self.label_progress_commandSpeed_3, 1, 0, 1, 1)
        self.label_feedSpeed2_2 = QtWidgets.QLabel(self.layoutWidget_4)
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(14)
        self.label_feedSpeed2_2.setFont(font)
        self.label_feedSpeed2_2.setStyleSheet("color: rgb(148, 148, 216); background-color: none;")
        self.label_feedSpeed2_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_feedSpeed2_2.setObjectName("label_feedSpeed2_2")
        self.infoLayout_feedSpeed_2.addWidget(self.label_feedSpeed2_2, 2, 0, 1, 1)

# ---------------------------------------------------- Steering ----------------------------------------------------------------
        self.circular_commandSpeed = QtWidgets.QFrame(self.frame)
        self.circular_commandSpeed.setGeometry(QtCore.QRect(20, 180, 240, 240))
        self.circular_commandSpeed.setStyleSheet("background-color: none;")
        self.circular_commandSpeed.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.circular_commandSpeed.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circular_commandSpeed.setObjectName("circular_commandSpeed")
        self.circularBg_commandSpeed_2 = QtWidgets.QFrame(self.circular_commandSpeed)
        self.circularBg_commandSpeed_2.setGeometry(QtCore.QRect(10, 10, 220, 220))
        self.circularBg_commandSpeed_2.setStyleSheet("QFrame{\n"
"    border-radius: 110px;    \n"
"    background-color: rgba(85, 85, 127, 100);\n"
"}")
        self.circularBg_commandSpeed_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.circularBg_commandSpeed_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circularBg_commandSpeed_2.setObjectName("circularBg_commandSpeed_2")
        self.animasi_commandSpeed_2 = QtWidgets.QFrame(self.circularBg_commandSpeed_2)
        self.animasi_commandSpeed_2.setGeometry(QtCore.QRect(0, 0, 220, 220))
        self.animasi_commandSpeed_2.setStyleSheet("QFrame{\n"
"    border-radius: 110px;\n"
"    border-strokes:inside;    \n"
"    background-color: qconicalgradient(cx:0.5, cy:0.5, angle:90, stop:0.400 rgba(85, 170, 255, 255), stop:0.395 rgba(255, 255, 255, 0));\n"
"}")
        # self.animasi_commandSpeed_2.setStyleSheet(self.new_animasi_steer)
        self.animasi_commandSpeed_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.animasi_commandSpeed_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.animasi_commandSpeed_2.setObjectName("animasi_commandSpeed_2")
        
        self.circularContainer_commandSpeed_2 = QtWidgets.QFrame(self.circular_commandSpeed)
        self.circularContainer_commandSpeed_2.setGeometry(QtCore.QRect(25, 25, 190, 190))
        self.circularContainer_commandSpeed_2.setBaseSize(QtCore.QSize(0, 0))
        self.circularContainer_commandSpeed_2.setStyleSheet("QFrame{\n"
"    border-radius: 95px;    \n"
"    background-color: rgb(58, 58, 102);\n"
"}")
        self.circularContainer_commandSpeed_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.circularContainer_commandSpeed_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.circularContainer_commandSpeed_2.setObjectName("circularContainer_commandSpeed_2")
        self.layoutWidget_2 = QtWidgets.QWidget(self.circularContainer_commandSpeed_2)
        self.layoutWidget_2.setGeometry(QtCore.QRect(10, 40, 171, 133))
        self.layoutWidget_2.setObjectName("layoutWidget_2")
        self.infoLayout_commandSpeed_2 = QtWidgets.QGridLayout(self.layoutWidget_2)
        self.infoLayout_commandSpeed_2.setContentsMargins(0, 0, 0, 0)
        self.infoLayout_commandSpeed_2.setObjectName("infoLayout_commandSpeed_2")
        self.label_commandSpeed_2 = QtWidgets.QLabel(self.layoutWidget_2)
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(10)
        self.label_commandSpeed_2.setFont(font)
        self.label_commandSpeed_2.setStyleSheet("color: #FFFFFF; background-color: none;")
        self.label_commandSpeed_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_commandSpeed_2.setObjectName("label_commandSpeed_2")
        self.infoLayout_commandSpeed_2.addWidget(self.label_commandSpeed_2, 0, 0, 1, 1)
        self.label_progress_commandSpeed_4 = QtWidgets.QLabel(self.layoutWidget_2)
        font = QtGui.QFont()
        font.setFamily("Roboto Thin")
        font.setPointSize(30)
        # Angka yg harus dirubah
        self.label_progress_commandSpeed_4.setFont(font)
        self.label_progress_commandSpeed_4.setStyleSheet("color: rgb(115, 185, 255); padding: 0px; background-color: none;")
        self.label_progress_commandSpeed_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_progress_commandSpeed_4.setIndent(-1)
        self.label_progress_commandSpeed_4.setObjectName("label_progress_commandSpeed_4")
        
        self.infoLayout_commandSpeed_2.addWidget(self.label_progress_commandSpeed_4, 1, 0, 1, 1)
        self.label_commanSpeed2_2 = QtWidgets.QLabel(self.layoutWidget_2)
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(14)
        self.label_commanSpeed2_2.setFont(font)
        self.label_commanSpeed2_2.setStyleSheet("color: rgb(148, 148, 216); background-color: none;")
        self.label_commanSpeed2_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_commanSpeed2_2.setObjectName("label_commanSpeed2_2")
        self.infoLayout_commandSpeed_2.addWidget(self.label_commanSpeed2_2, 2, 0, 1, 1)
        
# ----------------------------------------------- Object Detction Dashboard -------------------------------
        self.sensorLidar = QtWidgets.QFrame(self.frame)
        self.sensorLidar.setGeometry(QtCore.QRect(316, 20, 171, 81))
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
        self.sensorUltra.setGeometry(QtCore.QRect(280, 200, 241, 331))
        self.sensorUltra.setStyleSheet("background-color: none;")
        self.sensorUltra.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.sensorUltra.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorUltra.setObjectName("sensorUltra")
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
        self.sensorUltra_6 = QtWidgets.QFrame(self.sensorUltra)
        self.sensorUltra_6.setGeometry(QtCore.QRect(190, 180, 51, 31))
        self.sensorUltra_6.setStyleSheet("background-color: rgb(58, 58, 102);")
        self.sensorUltra_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorUltra_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorUltra_6.setObjectName("sensorUltra_6")
        self.sensorUltra_7 = QtWidgets.QFrame(self.sensorUltra)
        self.sensorUltra_7.setGeometry(QtCore.QRect(190, 100, 51, 31))
        self.sensorUltra_7.setStyleSheet("background-color: rgb(58, 58, 102);")
        self.sensorUltra_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorUltra_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorUltra_7.setObjectName("sensorUltra_7")                
        self.sensorUltra_8 = QtWidgets.QFrame(self.sensorUltra)
        self.sensorUltra_8.setGeometry(QtCore.QRect(190, 0, 51, 31))
        self.sensorUltra_8.setStyleSheet("background-color: rgb(58, 58, 102);")
        self.sensorUltra_8.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorUltra_8.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorUltra_8.setObjectName("sensorUltra_8")

        self.sensorCamera = QtWidgets.QFrame(self.frame)
        self.sensorCamera.setGeometry(QtCore.QRect(384, 100, 31, 41))
        self.sensorCamera.setStyleSheet("background-color: rgb(58, 58, 102);")
        self.sensorCamera.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sensorCamera.setFrameShadow(QtWidgets.QFrame.Raised)
        self.sensorCamera.setObjectName("sensorCamera")

# -------------------------------------------- Indicator Forward/Backward/Neutral ----------------------        
        self.switch_move = QtWidgets.QFrame(self.frame)
        self.switch_move.setGeometry(QtCore.QRect(220, 560, 381, 73))
        self.switch_move.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.switch_move.setFrameShadow(QtWidgets.QFrame.Raised)
        self.switch_move.setObjectName("switch_move")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.switch_move)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        
        self.switch_forward_2 = QtWidgets.QLabel(self.switch_move)
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.switch_forward_2.setFont(font)
        self.switch_forward_2.setStyleSheet("color:rgb(255, 255, 255)")
        self.switch_forward_2.setObjectName("switch_forward_2")
        self.horizontalLayout_4.addWidget(self.switch_forward_2)
        
        self.switch_netral_2 = QtWidgets.QLabel(self.switch_move)
        self.switch_netral_2.setEnabled(True)
        font = QtGui.QFont()
        font.setFamily("Noto Sans Telugu UI")
        font.setPointSize(30)
        font.setBold(True)
        font.setWeight(75)
        self.switch_netral_2.setFont(font)
        self.switch_netral_2.setStyleSheet("color: rgb(85, 255, 127);")
        self.switch_netral_2.setFrameShape(QtWidgets.QFrame.Box)
        self.switch_netral_2.setAlignment(QtCore.Qt.AlignCenter)
        self.switch_netral_2.setObjectName("switch_netral_2")
        self.horizontalLayout_4.addWidget(self.switch_netral_2)
        
        self.switch_backward_2 = QtWidgets.QLabel(self.switch_move)
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.switch_backward_2.setFont(font)
        self.switch_backward_2.setStyleSheet("color:rgb(255, 255, 255)")
        self.switch_backward_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.switch_backward_2.setObjectName("switch_backward_2")
        self.horizontalLayout_4.addWidget(self.switch_backward_2)
        
# --------------------------------------- Indicator Autonomous/Manual --------------------------------------------------        
        self.switch_otonom = QtWidgets.QFrame(self.frame)
        self.switch_otonom.setGeometry(QtCore.QRect(20, 10, 191, 41))
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
        
# ----------------------------------------- Mathplotlib ----------------------------------------------------------------        
        self.mathplotlib = QtWidgets.QFrame(self.frame)
        self.mathplotlib.setGeometry(QtCore.QRect(850, 70, 311, 491))
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


        self.line = QtWidgets.QFrame(self.frame)
        self.line.setGeometry(QtCore.QRect(30, 440, 261, 16))
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.line_2 = QtWidgets.QFrame(self.frame)
        self.line_2.setGeometry(QtCore.QRect(510, 440, 261, 16))
        self.line_2.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.button_path.raise_()
        self.button_ps.raise_()
        self.button_dock.raise_()
        self.sensorCamera.raise_()
        self.circular_feedSpeed.raise_()
        self.circular_commandSpeed.raise_()
        self.sensorLidar.raise_()
        self.switch_move.raise_()
        self.switch_otonom.raise_()
        self.mathplotlib.raise_()
        self.sensorUltra.raise_()
        self.imageCTT.raise_()
        self.line.raise_()
        self.line_2.raise_()
        self.button_check_steer.raise_()
        self.button_check_throttle.raise_()
        self.verticalLayout_7.addWidget(self.frame)
        self.stackedWidget.addWidget(self.page_1_dashboard)

# --------------------------------------- Page 2 ---------------------------------------------------------
        self.page_2_log = QtWidgets.QWidget()
        self.page_2_log.setObjectName("page_2_log")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.page_2_log)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.layout_OD_AI = QtWidgets.QHBoxLayout()
        self.layout_OD_AI.setObjectName("layout_OD_AI")
        self.frame_OD = QtWidgets.QFrame(self.page_2_log)
        self.frame_OD.setStyleSheet("background-color: rgb(77, 77, 127);")
        self.frame_OD.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_OD.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_OD.setObjectName("frame_OD")
        self.frame_label_OD = QtWidgets.QFrame(self.frame_OD)
        self.frame_label_OD.setGeometry(QtCore.QRect(8, 10, 271, 61))
        self.frame_label_OD.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_label_OD.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_label_OD.setObjectName("frame_label_OD")
        self.label_OD = QtWidgets.QLabel(self.frame_label_OD)
        self.label_OD.setGeometry(QtCore.QRect(10, 10, 251, 41))
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.label_OD.setFont(font)
        self.label_OD.setStyleSheet("color: rgb(255, 255, 255);")
        self.label_OD.setAlignment(QtCore.Qt.AlignCenter)
        self.label_OD.setObjectName("label_OD")
        self.verticalLayoutWidget_5 = QtWidgets.QWidget(self.frame_OD)
        self.verticalLayoutWidget_5.setGeometry(QtCore.QRect(140, 100, 131, 511))
        self.verticalLayoutWidget_5.setObjectName("verticalLayoutWidget_5")
        self.verticalLayout_12 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_5)
        self.verticalLayout_12.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_12.setObjectName("verticalLayout_12")

        self.ob_kamera = QtWidgets.QLCDNumber(self.verticalLayoutWidget_5)
        self.ob_kamera.setObjectName("ob_kamera")
        self.verticalLayout_12.addWidget(self.ob_kamera)
        self.ob_lidar = QtWidgets.QLCDNumber(self.verticalLayoutWidget_5)
        self.ob_lidar.setObjectName("ob_lidar")
        self.verticalLayout_12.addWidget(self.ob_lidar)
        self.ob_ultra1 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_5)
        self.ob_ultra1.setObjectName("ob_ultra1")
        self.verticalLayout_12.addWidget(self.ob_ultra1)
        self.ob_ultra2 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_5)
        self.ob_ultra2.setObjectName("ob_ultra2")
        self.verticalLayout_12.addWidget(self.ob_ultra2)
        self.ob_ultra3 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_5)
        self.ob_ultra3.setObjectName("ob_ultra3")
        self.verticalLayout_12.addWidget(self.ob_ultra3)
        self.ob_ultra4 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_5)
        self.ob_ultra4.setObjectName("ob_ultra4")
        self.verticalLayout_12.addWidget(self.ob_ultra4)
        self.ob_ultra5 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_5)
        self.ob_ultra5.setObjectName("ob_ultra5")
        self.verticalLayout_12.addWidget(self.ob_ultra5)
        self.ob_ultra6 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_5)
        self.ob_ultra6.setObjectName("ob_ultra6")
        self.verticalLayout_12.addWidget(self.ob_ultra6)
        self.ob_ultra7 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_5)
        self.ob_ultra7.setObjectName("ob_ultra7")
        self.verticalLayout_12.addWidget(self.ob_ultra7)
        self.ob_ultra8 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_5)
        self.ob_ultra8.setObjectName("ob_ultra8")
        self.verticalLayout_12.addWidget(self.ob_ultra8)
        self.ob_label_kamera = QtWidgets.QLabel(self.frame_OD)
        self.ob_label_kamera.setGeometry(QtCore.QRect(10, 110, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.ob_label_kamera.setFont(font)
        self.ob_label_kamera.setStyleSheet("color: rgb(255, 255, 255);")
        self.ob_label_kamera.setAlignment(QtCore.Qt.AlignCenter)
        self.ob_label_kamera.setObjectName("ob_label_kamera")
        self.ob_label_lidar = QtWidgets.QLabel(self.frame_OD)
        self.ob_label_lidar.setGeometry(QtCore.QRect(10, 160, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.ob_label_lidar.setFont(font)
        self.ob_label_lidar.setStyleSheet("color: rgb(255, 255, 255);")
        self.ob_label_lidar.setAlignment(QtCore.Qt.AlignCenter)
        self.ob_label_lidar.setObjectName("ob_label_lidar")
        self.ob_label_ultra = QtWidgets.QLabel(self.frame_OD)
        self.ob_label_ultra.setGeometry(QtCore.QRect(10, 210, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.ob_label_ultra.setFont(font)
        self.ob_label_ultra.setStyleSheet("color: rgb(255, 255, 255);")
        self.ob_label_ultra.setAlignment(QtCore.Qt.AlignCenter)
        self.ob_label_ultra.setObjectName("ob_label_ultra")
        self.layout_OD_AI.addWidget(self.frame_OD)
        self.frame_AI = QtWidgets.QFrame(self.page_2_log)
        self.frame_AI.setStyleSheet("background-color: rgb(77, 77, 127);")
        self.frame_AI.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_AI.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_AI.setObjectName("frame_AI")
        self.frame_label_AI = QtWidgets.QFrame(self.frame_AI)
        self.frame_label_AI.setGeometry(QtCore.QRect(9, 10, 271, 61))
        self.frame_label_AI.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_label_AI.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_label_AI.setObjectName("frame_label_AI")
        self.label_AI = QtWidgets.QLabel(self.frame_label_AI)
        self.label_AI.setGeometry(QtCore.QRect(10, 10, 251, 41))
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.label_AI.setFont(font)
        self.label_AI.setStyleSheet("color: rgb(255, 255, 255);")
        self.label_AI.setAlignment(QtCore.Qt.AlignCenter)
        self.label_AI.setObjectName("label_AI")
        self.verticalLayoutWidget_6 = QtWidgets.QWidget(self.frame_AI)
        self.verticalLayoutWidget_6.setGeometry(QtCore.QRect(150, 100, 121, 411))
        self.verticalLayoutWidget_6.setObjectName("verticalLayoutWidget_6")
        self.verticalLayout_13 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_6)
        self.verticalLayout_13.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_13.setObjectName("verticalLayout_13")
        self.ai_ecvlar = QtWidgets.QLCDNumber(self.verticalLayoutWidget_6)
        self.ai_ecvlar.setObjectName("ai_ecvlar")
        self.verticalLayout_13.addWidget(self.ai_ecvlar)
        self.ai_ecvlav = QtWidgets.QLCDNumber(self.verticalLayoutWidget_6)
        self.ai_ecvlav.setObjectName("ai_ecvlav")
        self.verticalLayout_13.addWidget(self.ai_ecvlav)
        self.ai_pt3 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_6)
        self.ai_pt3.setObjectName("ai_pt3")
        self.verticalLayout_13.addWidget(self.ai_pt3)
        self.ai_zc2 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_6)
        self.ai_zc2.setObjectName("ai_zc2")
        self.verticalLayout_13.addWidget(self.ai_zc2)
        self.ai_zc6 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_6)
        self.ai_zc6.setObjectName("ai_zc6")
        self.verticalLayout_13.addWidget(self.ai_zc6)
        self.ai_zc8 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_6)
        self.ai_zc8.setObjectName("ai_zc8")
        self.verticalLayout_13.addWidget(self.ai_zc8)
        self.ai_zc7 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_6)
        self.ai_zc7.setObjectName("ai_zc7")
        self.verticalLayout_13.addWidget(self.ai_zc7)
        self.ai_zc3_to_ecu = QtWidgets.QLCDNumber(self.verticalLayoutWidget_6)
        self.ai_zc3_to_ecu.setObjectName("ai_zc3_to_ecu")
        self.verticalLayout_13.addWidget(self.ai_zc3_to_ecu)
        self.ai_label_zc3_to_ecu = QtWidgets.QLabel(self.frame_AI)
        self.ai_label_zc3_to_ecu.setGeometry(QtCore.QRect(20, 470, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.ai_label_zc3_to_ecu.setFont(font)
        self.ai_label_zc3_to_ecu.setStyleSheet("color: rgb(255, 255, 255);")
        self.ai_label_zc3_to_ecu.setAlignment(QtCore.Qt.AlignCenter)
        self.ai_label_zc3_to_ecu.setObjectName("ai_label_zc3_to_ecu")
        self.ai_label_zc2 = QtWidgets.QLabel(self.frame_AI)
        self.ai_label_zc2.setGeometry(QtCore.QRect(20, 264, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.ai_label_zc2.setFont(font)
        self.ai_label_zc2.setStyleSheet("color: rgb(255, 255, 255);")
        self.ai_label_zc2.setAlignment(QtCore.Qt.AlignCenter)
        self.ai_label_zc2.setObjectName("ai_label_zc2")
        self.ai_label_pt3 = QtWidgets.QLabel(self.frame_AI)
        self.ai_label_pt3.setGeometry(QtCore.QRect(20, 213, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.ai_label_pt3.setFont(font)
        self.ai_label_pt3.setStyleSheet("color: rgb(255, 255, 255);")
        self.ai_label_pt3.setAlignment(QtCore.Qt.AlignCenter)
        self.ai_label_pt3.setObjectName("ai_label_pt3")
        self.ai_label_zc7 = QtWidgets.QLabel(self.frame_AI)
        self.ai_label_zc7.setGeometry(QtCore.QRect(20, 420, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.ai_label_zc7.setFont(font)
        self.ai_label_zc7.setStyleSheet("color: rgb(255, 255, 255);")
        self.ai_label_zc7.setAlignment(QtCore.Qt.AlignCenter)
        self.ai_label_zc7.setObjectName("ai_label_zc7")
        self.ai_label_zc8 = QtWidgets.QLabel(self.frame_AI)
        self.ai_label_zc8.setGeometry(QtCore.QRect(20, 367, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.ai_label_zc8.setFont(font)
        self.ai_label_zc8.setStyleSheet("color: rgb(255, 255, 255);")
        self.ai_label_zc8.setAlignment(QtCore.Qt.AlignCenter)
        self.ai_label_zc8.setObjectName("ai_label_zc8")
        self.ai_label_zc6 = QtWidgets.QLabel(self.frame_AI)
        self.ai_label_zc6.setGeometry(QtCore.QRect(20, 316, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.ai_label_zc6.setFont(font)
        self.ai_label_zc6.setStyleSheet("color: rgb(255, 255, 255);")
        self.ai_label_zc6.setAlignment(QtCore.Qt.AlignCenter)
        self.ai_label_zc6.setObjectName("ai_label_zc6")
        self.ai_label_ecvlav = QtWidgets.QLabel(self.frame_AI)
        self.ai_label_ecvlav.setGeometry(QtCore.QRect(20, 160, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.ai_label_ecvlav.setFont(font)
        self.ai_label_ecvlav.setStyleSheet("color: rgb(255, 255, 255);")
        self.ai_label_ecvlav.setAlignment(QtCore.Qt.AlignCenter)
        self.ai_label_ecvlav.setObjectName("ai_label_ecvlav")
        self.ai_label_ecvlar = QtWidgets.QLabel(self.frame_AI)
        self.ai_label_ecvlar.setGeometry(QtCore.QRect(20, 110, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.ai_label_ecvlar.setFont(font)
        self.ai_label_ecvlar.setStyleSheet("color: rgb(255, 255, 255);")
        self.ai_label_ecvlar.setAlignment(QtCore.Qt.AlignCenter)
        self.ai_label_ecvlar.setObjectName("ai_label_ecvlar")
        self.layout_OD_AI.addWidget(self.frame_AI)
        self.horizontalLayout_3.addLayout(self.layout_OD_AI)
        self.layout_DI = QtWidgets.QHBoxLayout()
        self.layout_DI.setObjectName("layout_DI")
        self.frame_DI = QtWidgets.QFrame(self.page_2_log)
        self.frame_DI.setStyleSheet("background-color: rgb(77, 77, 127);")
        self.frame_DI.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_DI.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_DI.setObjectName("frame_DI")
        self.frame_label_DI = QtWidgets.QFrame(self.frame_DI)
        self.frame_label_DI.setGeometry(QtCore.QRect(10, 10, 561, 61))
        self.frame_label_DI.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_label_DI.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_label_DI.setObjectName("frame_label_DI")
        self.label_DI = QtWidgets.QLabel(self.frame_label_DI)
        self.label_DI.setGeometry(QtCore.QRect(10, 10, 541, 41))
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.label_DI.setFont(font)
        self.label_DI.setStyleSheet("color: rgb(255, 255, 255);")
        self.label_DI.setAlignment(QtCore.Qt.AlignCenter)
        self.label_DI.setObjectName("label_DI")
        self.verticalLayoutWidget_3 = QtWidgets.QWidget(self.frame_DI)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(440, 100, 131, 511))
        self.verticalLayoutWidget_3.setObjectName("verticalLayoutWidget_3")
        self.layout_DI_2 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_3)
        self.layout_DI_2.setContentsMargins(0, 0, 0, 0)
        self.layout_DI_2.setObjectName("layout_DI_2")
        self.di_ys3_1_to_pvg = QtWidgets.QLCDNumber(self.verticalLayoutWidget_3)
        self.di_ys3_1_to_pvg.setObjectName("di_ys3_1_to_pvg")
        self.layout_DI_2.addWidget(self.di_ys3_1_to_pvg)
        self.di_ys3_2_to_pvg = QtWidgets.QLCDNumber(self.verticalLayoutWidget_3)
        self.di_ys3_2_to_pvg.setObjectName("di_ys3_2_to_pvg")
        self.layout_DI_2.addWidget(self.di_ys3_2_to_pvg)
        self.di_esv1415_to_pvg = QtWidgets.QLCDNumber(self.verticalLayoutWidget_3)
        self.di_esv1415_to_pvg.setObjectName("di_esv1415_to_pvg")
        self.layout_DI_2.addWidget(self.di_esv1415_to_pvg)
        self.di_ecv2_j19_to_pvg = QtWidgets.QLCDNumber(self.verticalLayoutWidget_3)
        self.di_ecv2_j19_to_pvg.setObjectName("di_ecv2_j19_to_pvg")
        self.layout_DI_2.addWidget(self.di_ecv2_j19_to_pvg)
        self.di_b01_nc = QtWidgets.QLCDNumber(self.verticalLayoutWidget_3)
        self.di_b01_nc.setObjectName("di_b01_nc")
        self.layout_DI_2.addWidget(self.di_b01_nc)
        self.di_c01_no = QtWidgets.QLCDNumber(self.verticalLayoutWidget_3)
        self.di_c01_no.setObjectName("di_c01_no")
        self.layout_DI_2.addWidget(self.di_c01_no)
        self.di_ls4 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_3)
        self.di_ls4.setObjectName("di_ls4")
        self.layout_DI_2.addWidget(self.di_ls4)
        self.di_av_to_actuator = QtWidgets.QLCDNumber(self.verticalLayoutWidget_3)
        self.di_av_to_actuator.setObjectName("di_av_to_actuator")
        self.layout_DI_2.addWidget(self.di_av_to_actuator)
        self.di_ar_to_actuator = QtWidgets.QLCDNumber(self.verticalLayoutWidget_3)
        self.di_ar_to_actuator.setObjectName("di_ar_to_actuator")
        self.layout_DI_2.addWidget(self.di_ar_to_actuator)
        self.di_st1 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_3)
        self.di_st1.setObjectName("di_st1")
        self.layout_DI_2.addWidget(self.di_st1)
        self.verticalLayoutWidget_4 = QtWidgets.QWidget(self.frame_DI)
        self.verticalLayoutWidget_4.setGeometry(QtCore.QRect(160, 100, 131, 511))
        self.verticalLayoutWidget_4.setObjectName("verticalLayoutWidget_4")
        self.layout_DI_1 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_4)
        self.layout_DI_1.setContentsMargins(0, 0, 0, 0)
        self.layout_DI_1.setObjectName("layout_DI_1")
        self.di_ps4 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_4)
        self.di_ps4.setObjectName("di_ps4")
        self.layout_DI_1.addWidget(self.di_ps4)
        self.di_ls2 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_4)
        self.di_ls2.setObjectName("di_ls2")
        self.layout_DI_1.addWidget(self.di_ls2)
        self.di_ps1 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_4)
        self.di_ps1.setObjectName("di_ps1")
        self.layout_DI_1.addWidget(self.di_ps1)
        self.di_pds1 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_4)
        self.di_pds1.setObjectName("di_pds1")
        self.layout_DI_1.addWidget(self.di_pds1)
        self.di_pds2 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_4)
        self.di_pds2.setObjectName("di_pds2")
        self.layout_DI_1.addWidget(self.di_pds2)
        self.di_ps2 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_4)
        self.di_ps2.setObjectName("di_ps2")
        self.layout_DI_1.addWidget(self.di_ps2)
        self.di_ts1 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_4)
        self.di_ts1.setObjectName("di_ts1")
        self.layout_DI_1.addWidget(self.di_ts1)
        self.di_ls1 = QtWidgets.QLCDNumber(self.verticalLayoutWidget_4)
        self.di_ls1.setObjectName("di_ls1")
        self.layout_DI_1.addWidget(self.di_ls1)
        self.di_as = QtWidgets.QLCDNumber(self.verticalLayoutWidget_4)
        self.di_as.setObjectName("di_as")
        self.layout_DI_1.addWidget(self.di_as)
        self.di_eb = QtWidgets.QLCDNumber(self.verticalLayoutWidget_4)
        self.di_eb.setObjectName("di_eb")
        self.layout_DI_1.addWidget(self.di_eb)
        self.di_label_ps4 = QtWidgets.QLabel(self.frame_DI)
        self.di_label_ps4.setGeometry(QtCore.QRect(20, 110, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_ps4.setFont(font)
        self.di_label_ps4.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_ps4.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_ps4.setObjectName("di_label_ps4")
        self.di_label_ls2 = QtWidgets.QLabel(self.frame_DI)
        self.di_label_ls2.setGeometry(QtCore.QRect(20, 161, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_ls2.setFont(font)
        self.di_label_ls2.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_ls2.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_ls2.setObjectName("di_label_ls2")
        self.di_label_ps1 = QtWidgets.QLabel(self.frame_DI)
        self.di_label_ps1.setGeometry(QtCore.QRect(20, 210, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_ps1.setFont(font)
        self.di_label_ps1.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_ps1.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_ps1.setObjectName("di_label_ps1")
        self.di_label_pds1 = QtWidgets.QLabel(self.frame_DI)
        self.di_label_pds1.setGeometry(QtCore.QRect(20, 266, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_pds1.setFont(font)
        self.di_label_pds1.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_pds1.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_pds1.setObjectName("di_label_pds1")
        self.di_label_pds2 = QtWidgets.QLabel(self.frame_DI)
        self.di_label_pds2.setGeometry(QtCore.QRect(20, 317, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_pds2.setFont(font)
        self.di_label_pds2.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_pds2.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_pds2.setObjectName("di_label_pds2")
        self.di_label_ps2 = QtWidgets.QLabel(self.frame_DI)
        self.di_label_ps2.setGeometry(QtCore.QRect(20, 369, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_ps2.setFont(font)
        self.di_label_ps2.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_ps2.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_ps2.setObjectName("di_label_ps2")
        self.di_label_ts1 = QtWidgets.QLabel(self.frame_DI)
        self.di_label_ts1.setGeometry(QtCore.QRect(20, 420, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_ts1.setFont(font)
        self.di_label_ts1.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_ts1.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_ts1.setObjectName("di_label_ts1")
        self.di_label_ls1 = QtWidgets.QLabel(self.frame_DI)
        self.di_label_ls1.setGeometry(QtCore.QRect(20, 470, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_ls1.setFont(font)
        self.di_label_ls1.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_ls1.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_ls1.setObjectName("di_label_ls1")
        self.di_label_as = QtWidgets.QLabel(self.frame_DI)
        self.di_label_as.setGeometry(QtCore.QRect(20, 525, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_as.setFont(font)
        self.di_label_as.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_as.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_as.setObjectName("di_label_as")
        self.di_label_eb = QtWidgets.QLabel(self.frame_DI)
        self.di_label_eb.setGeometry(QtCore.QRect(20, 573, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_eb.setFont(font)
        self.di_label_eb.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_eb.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_eb.setObjectName("di_label_eb")
        self.di_label_ys3_1_to_pvg = QtWidgets.QLabel(self.frame_DI)
        self.di_label_ys3_1_to_pvg.setGeometry(QtCore.QRect(310, 110, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_ys3_1_to_pvg.setFont(font)
        self.di_label_ys3_1_to_pvg.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_ys3_1_to_pvg.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_ys3_1_to_pvg.setObjectName("di_label_ys3_1_to_pvg")
        self.di_label_ys3_2_to_pvg = QtWidgets.QLabel(self.frame_DI)
        self.di_label_ys3_2_to_pvg.setGeometry(QtCore.QRect(310, 162, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_ys3_2_to_pvg.setFont(font)
        self.di_label_ys3_2_to_pvg.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_ys3_2_to_pvg.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_ys3_2_to_pvg.setObjectName("di_label_ys3_2_to_pvg")
        self.di_label_esv1415_to_pvg = QtWidgets.QLabel(self.frame_DI)
        self.di_label_esv1415_to_pvg.setGeometry(QtCore.QRect(304, 210, 131, 31))
        font = QtGui.QFont()
        font.setPointSize(13)
        self.di_label_esv1415_to_pvg.setFont(font)
        self.di_label_esv1415_to_pvg.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_esv1415_to_pvg.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_esv1415_to_pvg.setObjectName("di_label_esv1415_to_pvg")
        self.di_label_esv2_j19_to_pvg = QtWidgets.QLabel(self.frame_DI)
        self.di_label_esv2_j19_to_pvg.setGeometry(QtCore.QRect(305, 263, 131, 31))
        font = QtGui.QFont()
        font.setPointSize(13)
        self.di_label_esv2_j19_to_pvg.setFont(font)
        self.di_label_esv2_j19_to_pvg.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_esv2_j19_to_pvg.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_esv2_j19_to_pvg.setObjectName("di_label_esv2_j19_to_pvg")
        self.di_label_b01_nc = QtWidgets.QLabel(self.frame_DI)
        self.di_label_b01_nc.setGeometry(QtCore.QRect(310, 316, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_b01_nc.setFont(font)
        self.di_label_b01_nc.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_b01_nc.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_b01_nc.setObjectName("di_label_b01_nc")
        self.di_label_c01_no = QtWidgets.QLabel(self.frame_DI)
        self.di_label_c01_no.setGeometry(QtCore.QRect(310, 368, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_c01_no.setFont(font)
        self.di_label_c01_no.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_c01_no.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_c01_no.setObjectName("di_label_c01_no")
        self.di_label_ls4 = QtWidgets.QLabel(self.frame_DI)
        self.di_label_ls4.setGeometry(QtCore.QRect(306, 420, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_ls4.setFont(font)
        self.di_label_ls4.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_ls4.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_ls4.setObjectName("di_label_ls4")
        self.di_label_av_to_actuator = QtWidgets.QLabel(self.frame_DI)
        self.di_label_av_to_actuator.setGeometry(QtCore.QRect(300, 470, 131, 31))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.di_label_av_to_actuator.setFont(font)
        self.di_label_av_to_actuator.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_av_to_actuator.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_av_to_actuator.setObjectName("di_label_av_to_actuator")
        self.di_label_ar_to_actuator = QtWidgets.QLabel(self.frame_DI)
        self.di_label_ar_to_actuator.setGeometry(QtCore.QRect(300, 520, 131, 31))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.di_label_ar_to_actuator.setFont(font)
        self.di_label_ar_to_actuator.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_ar_to_actuator.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_ar_to_actuator.setObjectName("di_label_ar_to_actuator")
        self.di_label_st1 = QtWidgets.QLabel(self.frame_DI)
        self.di_label_st1.setGeometry(QtCore.QRect(300, 571, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.di_label_st1.setFont(font)
        self.di_label_st1.setStyleSheet("color: rgb(255, 255, 255);")
        self.di_label_st1.setAlignment(QtCore.Qt.AlignCenter)
        self.di_label_st1.setObjectName("di_label_st1")
        self.layout_DI.addWidget(self.frame_DI)
        self.horizontalLayout_3.addLayout(self.layout_DI)
        self.verticalLayout_6.addLayout(self.horizontalLayout_3)
        self.stackedWidget.addWidget(self.page_2_log)
        self.verticalLayout_5.addWidget(self.stackedWidget)
        self.horizontalLayout_2.addWidget(self.frame_pages)
        self.verticalLayout.addWidget(self.Content)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        self.stackedWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)



#     def __init__(self):
#         object.__init__(self)
#         self.setupUi

        
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
        self.ultra5 = msg.covariance[15]
        self.ultra6 = msg.covariance[16]
        self.ultra7 = msg.covariance[17]
        self.ultra8 = msg.covariance[18]

        self.ob_ultra1.display(self.ultra1)
        self.ob_ultra2.display(self.ultra2)
        self.ob_ultra3.display(self.ultra3)
        self.ob_ultra4.display(self.ultra4)
        self.ob_ultra5.display(self.ultra5)
        self.ob_ultra6.display(self.ultra6)
        self.ob_ultra7.display(self.ultra7)
        self.ob_ultra8.display(self.ultra8)

        # self.lidar_angle = msg.covariance[19]
        # self.lidar_range = msg.covariance[20]
        self.lidar_danger_flag = msg.covariance[21]

        if self.ultra1 < 100:
                self.sensorUltra_1.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_1.setStyleSheet("background-color: rgb(58, 58, 102);")     

        if self.ultra2 < 100:
                self.sensorUltra_2.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_2.setStyleSheet("background-color: rgb(58, 58, 102);")

        if self.ultra3 < 100:
                self.sensorUltra_3.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_3.setStyleSheet("background-color: rgb(58, 58, 102);")        

        if self.ultra4 < 100:
                self.sensorUltra_4.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_4.setStyleSheet("background-color: rgb(58, 58, 102);")

        if self.ultra5 < 100:
                self.sensorUltra_5.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_5.setStyleSheet("background-color: rgb(58, 58, 102);")

        if self.ultra6 < 100:
                self.sensorUltra_6.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_6.setStyleSheet("background-color: rgb(58, 58, 102);")

        if self.ultra7 < 100:
                self.sensorUltra_7.setStyleSheet("background-color: rgb(198, 59, 52);")
        else:
                self.sensorUltra_7.setStyleSheet("background-color: rgb(58, 58, 102);")

        if self.ultra8 < 100:
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
        # --------------------------------- Data Log AI and DI -------------------------
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

        self.ai_zc3_to_ecu.display(self.ZC3_to_ECU)
        self.ai_zc2.display(self.ZC2)
        self.ai_pt3.display(self.PT3)
        self.ai_zc7.display(self.ZC7)
        self.ai_zc8.display(self.ZC8)
        self.ai_zc6.display(self.ZC6)
        self.ai_ecvlav.display(self.ECVLAV)
        self.ai_ecvlar.display(self.ECVLAR)
        self.di_ps4.display(self.PS4)
        self.di_ls2.display(self.LS2)
        self.di_ps1.display(self.PS1)
        self.di_pds1.display(self.PDS1)
        self.di_pds2.display(self.PDS2)
        self.di_ps2.display(self.PS2)
        self.di_ts1.display(self.TS1)
        self.di_ls1.display(self.LS1)
        self.di_as.display(self.autonomous_switch)
        self.di_eb.display(self.emgs_button)
        self.di_ys3_1_to_pvg.display(self.YS31_to_PVG)
        self.di_ys3_2_to_pvg.display(self.YS32_to_PVG)
        self.di_esv1415_to_pvg.display(self.ESV1415_to_PVG)
        self.di_ecv2_j19_to_pvg.display(self.ECV2J19_to_PVG)
        self.di_b01_nc.display(self.B01)
        self.di_c01_no.display(self.C01)
        self.di_ls4.display(self.LS4)
        self.di_av_to_actuator.display(self.ESVDLAR_ENA)
        self.di_ar_to_actuator.display(self.ECVLAR_ENA)
        self.di_st1.display(self.ST1)
               

        # -------------------------------- Control Mode ---------------------------------
        # self.lcd_command_steer.display(np.degrees(self.cs_lat)) #show command steer in degree
        # self.lcd_command_throttle.display(self.cs_long)
        if self.mode_auto == 1: self.mode1()
        elif self.mode_auto == 2: self.mode2()
        elif self.mode_auto == 3: self.mode3()
        elif self.mode_auto == 4: self.mode4()
        elif self.mode_auto == 5: self.mode5()
        elif self.mode_auto == 0: self.mode0()

        self.display_dash()
        self.animasi()        

    def mode0(self):       #MODE0
        self.mode_auto = 0
        self.mode_auto_bef = self.mode_auto
        self.label_switch_otonom.setText("MANUAL")

    def mode1(self):       #MODE1
        # check state before
        print("masuk steer")
        if self.mode_auto_bef == 0: 
            self.mode_auto = 1
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        # check whether the command has been complete or not
        if self.cs_lamp == 1:
            self.label_switch_otonom.setText("AUTONOMOUS")            
        else:
            # bring back to stand by mode
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
            self.label_switch_otonom.setText("MANUAL")

        # publish message
        self.idc_lamp()

    def mode2(self): #MODE2
        # check state before
        print("masuk throttle")
        if self.mode_auto_bef == 0: 
            self.mode_auto = 2
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        # check whether the command has been complete or not
        if self.cs_lamp == 1:
            self.label_switch_otonom.setText("AUTONOMOUS")            
        else:
            # bring back to stand by mode
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
            self.label_switch_otonom.setText("MANUAL")
        # publish message
        self.idc_lamp()

    def mode3(self): #MODE3
        # check state before
        print("masuk path")
        if self.mode_auto_bef == 0: 
            self.mode_auto = 3
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        # check whether the command has been complete or not
        if self.cs_lamp == 1:
            self.label_switch_otonom.setText("AUTONOMOUS")            
        else:
            # bring back to stand by mode
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
            self.label_switch_otonom.setText("MANUAL")
        # publish message
        self.idc_lamp()

    def mode4(self):  #MODE4        
        # check state before
        # print("masuk point")
        if self.mode_auto_bef == 0: 
            self.mode_auto = 4
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        # check whether the command has been complete or not
        if self.cs_lamp == 1:
            self.label_switch_otonom.setText("AUTONOMOUS")            
        else:
            # bring back to stand by mode
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
            self.label_switch_otonom.setText("MANUAL")
        # publish message
        self.idc_lamp()      

    def mode5(self):        #MODE5
        # check state before
        print("masuk docking")
        if self.mode_auto_bef == 0: 
            self.mode_auto = 5
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        # check whether the command has been complete or not
        if self.cs_lamp == 1:
            self.label_switch_otonom.setText("AUTONOMOUS")            
        else:
            # bring back to stand by mode
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
            self.label_switch_otonom.setText("MANUAL")
        # publish message
        self.idc_lamp()

    def idc_lamp(self):            
        self.msg.pose.covariance[0] = self.mode_auto
        self.pub.publish(self.msg)

    def display_dash(self):
        # self.callback_data_collector()
        self.label_progress_commandSpeed_3.setText(str(round(self.V_now*3.6,1)))        #Speed
        self.label_progress_commandSpeed_4.setText(str(round(self.steer_now,1)))         #Steering

    def animasi(self):
        # self.callback_data_collector()            
        # ------------------------ Animasi Steering ---------------------------------------------------       
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
        self.progress_steer = (40 - self.value_steer)/40.0
        
        # New Value
        self.stop_1_steer = str(self.progress_steer - 0.001)
        self.stop_2_steer = str(self.progress_steer)

        # New Stylesheet
        self.new_animasi_steer = self.stylesheet_steer.replace("{STOP_1_steer}", self.stop_1_steer).replace("{STOP_2_steer}", self.stop_2_steer)

        # Apply stylesheet
        # self.animasi_commandSpeed_2.setStyleSheet(self.new_animasi_steer)

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
        self.progress_speed = (100 - self.value_speed)/100.0
        
        # New Value
        self.stop_1_speed = str(self.progress_speed - 0.001)
        self.stop_2_speed = str(self.progress_speed)

        # New Stylesheet
        self.new_animasi_speed = self.stylesheet_speed.replace("{STOP_1_speed}", self.stop_1_speed).replace("{STOP_2_speed}", self.stop_2_speed)

        # Apply stylesheet
        # self.animasi_feedSpeed_2.setStyleSheet(self.new_animasi_speed)
# -------------------------------- Mathplotlib -------------------------------------------------------
    def plot_init(self):
        # plot init
        self._dynamic_ax.set_xlim(min(self.X_utm)-50, max(self.X_utm) + 20)
        self._dynamic_ax.set_ylim(min(self.Y_utm)-10, max(self.Y_utm) + 10)
        self._dynamic_ax.set_xlabel("X_utm(m)")
        self._dynamic_ax.set_ylabel("Y_utm(m)")
        # self._dynamic_ax.axis("equal")
        # Set up plot variable
        self._pos, = self._dynamic_ax.plot([],[],'.',label="AGV position")
        self._bound, = self._dynamic_ax.plot([], [], "k")
        self._trailer_plot, = self._dynamic_ax.plot([], [], "g")
        self._head_plot, = self._dynamic_ax.plot([], [], "g")
        self._waypoints, = self._dynamic_ax.plot([], [], "r")
        self.l_dock_, = self._dynamic_ax.plot([], [], "m")
        self.r_dock_, = self._dynamic_ax.plot([], [], "m")
        self.low_dock_, = self._dynamic_ax.plot([], [], "m")
        self._target_PS, = self._dynamic_ax.plot([], [], ".")
        self._target_dock, = self._dynamic_ax.plot([], [], ".")
        self._waypoint_path_zero, = self._dynamic_ax.plot([], [], ".", label="start path follow")
        # self._waypoint_path_zero, = self._dynamic_ax.plot([], [], ".", label="start ps")
        self.xtodock, self.ytodock = 558098.5399999999, 9158001.32#min(self.X_utm) + 14.5, min(self.Y_utm) + 70
        self.start_to_dock, = plt.plot([],[], ".", label="start point")
        self._dynamic_ax.legend(loc=2, prop={'size': 6})

    def _update_canvas(self):
        # self.plot_init()
        if self.mode_auto == 3: 
            self._waypoints.set_data(self.waypoints[:,0],self.waypoints[:,1])
            self._waypoints.figure.canvas.draw()
        elif self.mode_auto == 4: 
            self._target_PS.set_data(self.PSbackX, self.PSbackY)
            self._target_PS.figure.canvas.draw()
        elif self.mode_auto == 5:
            self._target_dock.set_data(self.PSdockX, self.PSdockY)
            self._target_dock.figure.canvas.draw()
        else: 
            # clear the plotting area
            self._dynamic_ax.cla()
            self.plot_init()
        
        # set data_update
        self._pos.set_data(self.X_now, self.Y_now)
        self._bound.set_data(self.X_utm, self.Y_utm)
        # AV_Trailer = self.Trailer_(self.X_now, self.Y_now, self.yaw_now)
        AV_trailer = self.Trailer_(self.X_now + 11.291/2 * np.cos(self.yaw_now)\
            , self.Y_now + 11.291/2 * np.sin(self.yaw_now), self.yaw_now)
        x3,y3 = Polygon(AV_trailer).exterior.xy
        self._trailer_plot.set_data(x3,y3)
        # AV_Head = self.head_(self.X_now, self.Y_now, self.yaw_now, self.yaw_head)
        AV_head = self.head_(self.X_now + 11.291/2 * np.cos(self.yaw_now)\
            , self.Y_now + 11.291/2 * np.sin(self.yaw_now), self.yaw_now, self.yaw_head)
        x4,y4 = Polygon(AV_head).exterior.xy
        self._head_plot.set_data(x4,y4)
        # print(x4,y4)
        wpx = self.waypoints[:,0]
        wpy = self.waypoints[:,1]
        # self._waypoint_path_zero.set_data(wpx[0],wpy[0])
        self._waypoint_path_zero.set_data(self.xtodock, self.ytodock)
        # create docking station
        l_dock_ = self.l_dock_pos
        r_dock_ = self.r_dock_pos
        low_dock_ = self.low_dock_pos
        l_dock_box = self.dock_(l_dock_[0],l_dock_[1],l_dock_[2],l_dock_[3],l_dock_[4])
        x5,y5 = Polygon(l_dock_box).exterior.xy
        # print(x5,y5)
        self.l_dock_.set_data(x5,y5)
        r_dock_box = self.dock_(r_dock_[0],r_dock_[1],r_dock_[2],r_dock_[3],r_dock_[4])
        x6,y6 = Polygon(r_dock_box).exterior.xy
        self.r_dock_.set_data(x6,y6)
        low_dock_box = self.dock_(low_dock_[0],low_dock_[1],low_dock_[2],low_dock_[3],low_dock_[4])
        x7,y7 = Polygon(low_dock_box).exterior.xy
        self.low_dock_.set_data(x7,y7)

        # draw the data visualization
        self.start_to_dock.figure.canvas.draw()
        self._pos.figure.canvas.draw()
        self._trailer_plot.figure.canvas.draw()
        self._head_plot.figure.canvas.draw()
        self._bound.figure.canvas.draw()
        self._waypoint_path_zero.figure.canvas.draw()
        self.l_dock_.figure.canvas.draw()
        self.r_dock_.figure.canvas.draw()
        self.low_dock_.figure.canvas.draw()
        # print("hereeeee")
        plt.axis("equal")
        # self._dynamic_ax.legend()

    def dock_(self, X_pos, Y_pos, yaw_,l_, w_): #(x position, y position, yaw angle, length, width)
        # Create polygon coordinates
        transform_coor = [[X_pos-l_/2*np.cos(yaw_)+w_/2*np.sin(yaw_),Y_pos-l_/2*np.sin(yaw_)-w_/2*np.cos(yaw_)],
                [X_pos-l_/2*np.cos(yaw_)-w_/2*np.sin(yaw_),Y_pos-l_/2*np.sin(yaw_)+w_/2*np.cos(yaw_)],
                [X_pos+l_/2*np.cos(yaw_)-w_/2*np.sin(yaw_),Y_pos+l_/2*np.sin(yaw_)+w_/2*np.cos(yaw_)],
                [X_pos+l_/2*np.cos(yaw_)+w_/2*np.sin(yaw_),Y_pos+l_/2*np.sin(yaw_)-w_/2*np.cos(yaw_)]]
        return transform_coor
    
    def Trailer_(self, X_pos, Y_pos, yaw_): #(x position, y position, yaw angle, length, width)
        # Create polygon coordinates
        transform_coor = [[X_pos-self.l_t/2*np.cos(yaw_)+self.w_t/2*np.sin(yaw_),Y_pos-self.l_t/2*np.sin(yaw_)-self.w_t/2*np.cos(yaw_)],
                [X_pos-self.l_t/2*np.cos(yaw_)-self.w_t/2*np.sin(yaw_),Y_pos-self.l_t/2*np.sin(yaw_)+self.w_t/2*np.cos(yaw_)],
                [X_pos+self.l_t/2*np.cos(yaw_)-self.w_t/2*np.sin(yaw_),Y_pos+self.l_t/2*np.sin(yaw_)+self.w_t/2*np.cos(yaw_)],
                [X_pos+self.l_t/2*np.cos(yaw_)+self.w_t/2*np.sin(yaw_),Y_pos+self.l_t/2*np.sin(yaw_)-self.w_t/2*np.cos(yaw_)]]
        return transform_coor  

    def head_(self, X_pos_t, Y_pos_t, yaw_t, yaw_h): 
        # create head center point
        h_center = [X_pos_t + (self.l_t/2-0.755)*np.cos(yaw_t), Y_pos_t + (self.l_t/2-0.755)*np.sin(yaw_t)]
        h_transform = [[h_center[0]-self.l_h/2*np.cos(yaw_h)+self.w_h/2*np.sin(yaw_h),h_center[1]-self.l_h/2*np.sin(yaw_h)-self.w_h/2*np.cos(yaw_h)],
                [h_center[0]-self.l_h/2*np.cos(yaw_h)-self.w_h/2*np.sin(yaw_h),h_center[1]-self.l_h/2*np.sin(yaw_h)+self.w_h/2*np.cos(yaw_h)],
                [h_center[0]+self.l_h/2*np.cos(yaw_h)-self.w_h/2*np.sin(yaw_h),h_center[1]+self.l_h/2*np.sin(yaw_h)+self.w_h/2*np.cos(yaw_h)],
                [h_center[0]+self.l_h/2*np.cos(yaw_h)+self.w_h/2*np.sin(yaw_h),h_center[1]+self.l_h/2*np.sin(yaw_h)-self.w_h/2*np.cos(yaw_h)]]
        return h_transform

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.Btn_menu.setText(_translate("MainWindow", "MENU"))
        self.label_title.setText(_translate("MainWindow", "AUTONOMOUS GUIDE VEHICLE (AGV) - CTT MERAH PUTIH"))
        self.btn_page_1.setText(_translate("MainWindow", "Dash"))
        self.btn_page_2.setText(_translate("MainWindow", "Log"))
        self.button_path.setText(_translate("MainWindow", "PATH"))
        self.button_ps.setText(_translate("MainWindow", "POINT"))
        self.button_dock.setText(_translate("MainWindow", "DOCKING"))
        self.label_feedSpeed_2.setText(_translate("MainWindow", "<html><head/><body><p>SPEED</p></body></html>"))
        self.label_progress_commandSpeed_3.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:50pt;\">66</span></p></body></html>"))
        self.label_feedSpeed2_2.setText(_translate("MainWindow", "<html><head/><body><p>KM/H</p></body></html>"))
        self.label_commandSpeed_2.setText(_translate("MainWindow", "<html><head/><body><p>STEERING</p></body></html>"))
        self.label_progress_commandSpeed_4.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:50pt;\">60</span></p></body></html>"))
        self.label_commanSpeed2_2.setText(_translate("MainWindow", "<html><head/><body><p>DEGREE</p></body></html>"))
        self.switch_forward_2.setText(_translate("MainWindow", "FORWARD"))
        self.switch_netral_2.setText(_translate("MainWindow", "N"))
        self.switch_backward_2.setText(_translate("MainWindow", " BACKWARD"))
        self.label_switch_otonom.setText(_translate("MainWindow", "MANUAL"))
        self.button_check_steer.setText(_translate("MainWindow", "CHECK STEER"))
        self.button_check_throttle.setText(_translate("MainWindow", "CHECK THROTTLE"))
        self.label_OD.setText(_translate("MainWindow", "OBJECT DETECTION"))
        self.ob_label_kamera.setText(_translate("MainWindow", "KAMERA"))
        self.ob_label_lidar.setText(_translate("MainWindow", "LiDAR"))
        self.ob_label_ultra.setText(_translate("MainWindow", "ULTRASONIK"))
        self.label_AI.setText(_translate("MainWindow", "ANALOG INPUT"))
        self.ai_label_zc3_to_ecu.setText(_translate("MainWindow", "ZC3 to ECU"))
        self.ai_label_zc2.setText(_translate("MainWindow", "ZC2"))
        self.ai_label_pt3.setText(_translate("MainWindow", "PT3"))
        self.ai_label_zc7.setText(_translate("MainWindow", "ZC7"))
        self.ai_label_zc8.setText(_translate("MainWindow", "ZC8"))
        self.ai_label_zc6.setText(_translate("MainWindow", "ZC6"))
        self.ai_label_ecvlav.setText(_translate("MainWindow", "ECVLAV"))
        self.ai_label_ecvlar.setText(_translate("MainWindow", "ECVLAR"))
        self.label_DI.setText(_translate("MainWindow", "DIGITAL INPUT"))
        self.di_label_ps4.setText(_translate("MainWindow", "PS4"))
        self.di_label_ls2.setText(_translate("MainWindow", "LS2"))
        self.di_label_ps1.setText(_translate("MainWindow", "PS1"))
        self.di_label_pds1.setText(_translate("MainWindow", "PDS1"))
        self.di_label_pds2.setText(_translate("MainWindow", "PDS2"))
        self.di_label_ps2.setText(_translate("MainWindow", "PS2"))
        self.di_label_ts1.setText(_translate("MainWindow", "TS1"))
        self.di_label_ls1.setText(_translate("MainWindow", "LS1"))
        self.di_label_as.setText(_translate("MainWindow", "AUTO SWITCH"))
        self.di_label_eb.setText(_translate("MainWindow", "EMERGENCY"))
        self.di_label_ys3_1_to_pvg.setText(_translate("MainWindow", "YS3(1) to PVG"))
        self.di_label_ys3_2_to_pvg.setText(_translate("MainWindow", "YS3(2) to PVG"))
        self.di_label_esv1415_to_pvg.setText(_translate("MainWindow", "ESV1415 to PVG"))
        self.di_label_esv2_j19_to_pvg.setText(_translate("MainWindow", "ECV2 J19 to PVG"))
        self.di_label_b01_nc.setText(_translate("MainWindow", "B01 NC"))
        self.di_label_c01_no.setText(_translate("MainWindow", "C01 NO"))
        self.di_label_ls4.setText(_translate("MainWindow", "LS4"))
        self.di_label_av_to_actuator.setText(_translate("MainWindow", "AV to ACTUATOR"))
        self.di_label_ar_to_actuator.setText(_translate("MainWindow", "AR to ACTUATOR"))
        self.di_label_st1.setText(_translate("MainWindow", "ST1"))

if __name__ == "__main__":
#     import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

