#! /usr/bin/env python3
# Created by: GAN
from socket import CMSG_LEN
import sys
import os

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QWidget, QLabel, QLineEdit
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtCore import QSize 


import qdarkstyle

class Ui_MainWindow(object):
    
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1280, 720)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        app.setStyleSheet(qdarkstyle.load_stylesheet_pyside2())

        # Insiasi nilai awal
        self.pub = rospy.Publisher('idc_lamp', PoseWithCovarianceStamped, queue_size=10)
        rospy.init_node('pub_lamp')                       
        self.sub = rospy.Subscriber('/control_algorithm', PoseWithCovarianceStamped, self.sub_control)
        self.msg = PoseWithCovarianceStamped()
        self.cs_lamp = 0
        self.mode_auto = 0
        self.mode_auto_bef = 0

        self.lcd_set_steer = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcd_set_steer.setGeometry(QtCore.QRect(56, 190, 231, 111))
        self.lcd_set_steer.setObjectName("lcd_set_steer")
        self.lcd_feed_steer = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcd_feed_steer.setGeometry(QtCore.QRect(456, 190, 231, 111))
        self.lcd_feed_steer.setObjectName("lcd_feed_steer")
        self.lcd_set_throttle = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcd_set_throttle.setGeometry(QtCore.QRect(70, 427, 231, 111))
        self.lcd_set_throttle.setObjectName("lcd_set_throttle")
        self.lcd_feed_throttle = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcd_feed_throttle.setGeometry(QtCore.QRect(460, 427, 231, 111))
        self.lcd_feed_throttle.setObjectName("lcd_feed_throttle")
        
        self.label_steer = QtWidgets.QLabel(self.centralwidget)
        self.label_steer.setGeometry(QtCore.QRect(60, 150, 221, 21))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_steer.setFont(font)
        self.label_steer.setAlignment(QtCore.Qt.AlignCenter)
        self.label_steer.setObjectName("label_steer")
        self.label_feed_steer = QtWidgets.QLabel(self.centralwidget)
        self.label_feed_steer.setGeometry(QtCore.QRect(460, 150, 231, 21))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_feed_steer.setFont(font)
        self.label_feed_steer.setAlignment(QtCore.Qt.AlignCenter)
        self.label_feed_steer.setObjectName("label_feed_steer")
        self.label_throttle = QtWidgets.QLabel(self.centralwidget)
        self.label_throttle.setGeometry(QtCore.QRect(70, 381, 231, 21))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_throttle.setFont(font)
        self.label_throttle.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_throttle.setAlignment(QtCore.Qt.AlignCenter)
        self.label_throttle.setObjectName("label_throttle")
        self.label_feed_throttle = QtWidgets.QLabel(self.centralwidget)
        self.label_feed_throttle.setGeometry(QtCore.QRect(460, 380, 231, 21))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_feed_throttle.setFont(font)
        self.label_feed_throttle.setAlignment(QtCore.Qt.AlignCenter)
        self.label_feed_throttle.setObjectName("label_feed_throttle")
        
         
        self.mode = QtWidgets.QFrame(self.centralwidget)
        self.mode.setGeometry(QtCore.QRect(770, 70, 491, 481))
        self.mode.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.mode.setFrameShadow(QtWidgets.QFrame.Raised)
        self.mode.setObjectName("mode")
        self.label_mode = QtWidgets.QLabel(self.mode)
        self.label_mode.setGeometry(QtCore.QRect(60, 20, 231, 21))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_mode.setFont(font)
        self.label_mode.setAlignment(QtCore.Qt.AlignCenter)
        self.label_mode.setObjectName("label_mode")

        # -------------- Code of autonomous mode ---------------------
        self.button_steer = QtWidgets.QPushButton(self.mode)
        self.button_steer.setGeometry(QtCore.QRect(50, 70, 241, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.button_steer.setFont(font)
        self.button_steer.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.button_steer.setIconSize(QtCore.QSize(16, 16))
        self.button_steer.setObjectName("button_steer")
        self.button_steer.clicked.connect(self.mode1)

        self.button_throttle = QtWidgets.QPushButton(self.mode)
        self.button_throttle.setGeometry(QtCore.QRect(50, 150, 241, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.button_throttle.setFont(font)
        self.button_throttle.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.button_throttle.setObjectName("button_throttle")
        self.button_throttle.clicked.connect(self.mode2)

        self.path_following = QtWidgets.QPushButton(self.mode)
        self.path_following.setGeometry(QtCore.QRect(50, 230, 241, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.path_following.setFont(font)
        self.path_following.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.path_following.setIconSize(QtCore.QSize(16, 16))
        self.path_following.setObjectName("path_following")
        self.path_following.clicked.connect(self.mode3)

        self.point_stabilization = QtWidgets.QPushButton(self.mode)
        self.point_stabilization.setGeometry(QtCore.QRect(50, 310, 241, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.point_stabilization.setFont(font)
        self.point_stabilization.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.point_stabilization.setIconSize(QtCore.QSize(16, 16))
        self.point_stabilization.setObjectName("point_stabilization")
        self.point_stabilization.clicked.connect(self.mode4)

        self.docking = QtWidgets.QPushButton(self.mode)
        self.docking.setGeometry(QtCore.QRect(50, 390, 241, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.docking.setFont(font)
        self.docking.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.docking.setIconSize(QtCore.QSize(16, 16))
        self.docking.setObjectName("docking")
        self.docking.clicked.connect(self.mode5)

        self.label_led = QtWidgets.QLabel(self.mode)
        self.label_led.setGeometry(QtCore.QRect(344, 20, 111, 21))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_led.setFont(font)
        self.label_led.setAlignment(QtCore.Qt.AlignCenter)
        self.label_led.setObjectName("label_indicator")
        self.label_led_2 = QtWidgets.QLabel(self.mode)
        self.label_led_2.setGeometry(QtCore.QRect(346, 47, 111, 21))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_led_2.setFont(font)
        self.label_led_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_led_2.setObjectName("label_indicator_2")

        self.led_indicator = QtWidgets.QFrame(self.mode)
        self.led_indicator.setGeometry(QtCore.QRect(350, 230, 101, 101))
        self.led_indicator.setStyleSheet("border-radius: 50px;    \n"
"background-color: rgb(58, 58, 102);")
        self.led_indicator.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.led_indicator.setFrameShadow(QtWidgets.QFrame.Raised)
        self.led_indicator.setObjectName("led_docking")
        # ------------- End Code of autonomous mode ----------------------        

        self.label_hmi = QtWidgets.QLabel(self.centralwidget)
        self.label_hmi.setGeometry(QtCore.QRect(160, 60, 441, 51))
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.label_hmi.setFont(font)
        self.label_hmi.setAlignment(QtCore.Qt.AlignCenter)
        self.label_hmi.setObjectName("human_machine_interface")
        
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1280, 25))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def sub_control(self, msg2):        
        self.cs_lat = msg2.pose.covariance[0]    #steering angle in rad
        self.cs_long = msg2.pose.covariance[1]   #speed set point in m/s
        self.cs_brake = msg2.pose.covariance[2]  #braking percentage
        self.cs_lamp = msg2.pose.covariance[3]   #autonomous lamp
        # self.cs_lamp = 1
        print(self.cs_lamp, self.mode_auto, self.mode_auto_bef)
        # check the condition: 
        if self.mode_auto == 1: self.mode1()
        elif self.mode_auto == 2: self.mode2()
        elif self.mode_auto == 3: self.mode3()
        elif self.mode_auto == 4: self.mode4()
        elif self.mode_auto == 5: self.mode5()
             
    def mode1(self):
        # check state before
        if self.mode_auto_bef == 0: 
            self.mode_auto = 1
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        # check whether the command has been complete or not
        if self.cs_lamp == 1:
            self.led_indicator.setStyleSheet("border-radius: 50px;    \n"
"background-color: rgb(198, 59, 52);")
        else:
            # bring back to stand by mode
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
            self.led_indicator.setStyleSheet("border-radius: 50px;    \n"
"background-color: rgb(58, 58, 102);")
        # publish message
        self.idc_lamp()

    def mode2(self):
        # check state before
        if self.mode_auto_bef == 0: 
            self.mode_auto = 2
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        # check whether the command has been complete or not
        if self.cs_lamp == 1:
            self.led_indicator.setStyleSheet("border-radius: 50px;    \n"
"background-color: rgb(198, 59, 52);")
        else:
            # bring back to stand by mode
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
            self.led_indicator.setStyleSheet("border-radius: 50px;    \n"
"background-color: rgb(58, 58, 102);")
        # publish message
        self.idc_lamp()

    def mode3(self):
        # check state before
        if self.mode_auto_bef == 0: 
            self.mode_auto = 3
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        # check whether the command has been complete or not
        if self.cs_lamp == 1:
            self.led_indicator.setStyleSheet("border-radius: 50px;    \n"
"background-color: rgb(198, 59, 52);")
        else:
            # bring back to stand by mode
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
            self.led_indicator.setStyleSheet("border-radius: 50px;    \n"
"background-color: rgb(58, 58, 102);")
        # publish message
        self.idc_lamp()  

    def mode4(self):
        # check state before
        if self.mode_auto_bef == 0: 
            self.mode_auto = 4
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        # check whether the command has been complete or not
        if self.cs_lamp == 1:
            self.led_indicator.setStyleSheet("border-radius: 50px;    \n"
"background-color: rgb(198, 59, 52);")
        else:
            # bring back to stand by mode
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
            self.led_indicator.setStyleSheet("border-radius: 50px;    \n"
"background-color: rgb(58, 58, 102);")
        # publish message
        self.idc_lamp()

    def mode5(self):
        # check state before
        if self.mode_auto_bef == 0: 
            self.mode_auto = 5
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        # check whether the command has been complete or not
        if self.cs_lamp == 1:
            self.led_indicator.setStyleSheet("border-radius: 50px;    \n"
"background-color: rgb(198, 59, 52);")
        else:
            # bring back to stand by mode
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
            self.led_indicator.setStyleSheet("border-radius: 50px;    \n"
"background-color: rgb(58, 58, 102);")
        # publish message
        self.idc_lamp()

    def idc_lamp(self):                    
        self.msg.pose.covariance[0] = self.mode_auto
        self.pub.publish(self.msg)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_steer.setText(_translate("MainWindow", "STEERING COMMAND"))
        self.label_feed_steer.setText(_translate("MainWindow", "FEEDBACK STEERING"))
        self.label_throttle.setText(_translate("MainWindow", "THROTTLE COMMAND"))
        self.label_feed_throttle.setText(_translate("MainWindow", "FEEDBACK SPEED"))
        self.label_mode.setText(_translate("MainWindow", "MODE"))
        self.label_led.setText(_translate("MainWindow", "INDIKATOR"))
        self.label_led_2.setText(_translate("MainWindow", "OTONOM"))
        self.button_steer.setText(_translate("MainWindow", "CHECK STEERING"))
        self.button_throttle.setText(_translate("MainWindow", "CHECK THROTTLE"))
        self.docking.setText(_translate("MainWindow", "DOCKING"))
        self.path_following.setText(_translate("MainWindow", "PATH FOLLOWING"))
        self.point_stabilization.setText(_translate("MainWindow", "POINT STABILIZATION"))
        self.label_hmi.setText(_translate("MainWindow", "HUMAN MACHINE INTERFACE"))


if __name__ == "__main__":
    # import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    # print("testttts")
    sys.exit(app.exec_())

# rospy.spin()