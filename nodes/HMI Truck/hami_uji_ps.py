#!/usr/bin/env python3
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
# from agv_ctt.msg import data_conv, dbw_pubsub

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1000, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        # Subscribe ROS
        rospy.init_node('node_hmi')
        self.sub = rospy.Subscriber('/data_conv', PoseWithCovarianceStamped, self.callback_data_conv)
        rospy.Subscriber('/control_algorithm', PoseWithCovarianceStamped, self.callback_controlAlgorithm)

        #Publish ROS
        self.pub = rospy.Publisher('/idc_lamp', PoseWithCovarianceStamped, queue_size=10)
        self.msg = PoseWithCovarianceStamped()

        #Insisasi awal
        self.cs_lamp = 1
        self.mode_auto = 0
        self.mode_auto_bef = 0
        self.steer_now = 0
        self.V_now = 0

        self.label_steering = QtWidgets.QLabel(self.centralwidget)
        self.label_steering.setGeometry(QtCore.QRect(100, 40, 181, 61))
        font = QtGui.QFont()
        font.setPointSize(27)
        font.setBold(True)
        font.setWeight(75)
        self.label_steering.setFont(font)
        self.label_steering.setAlignment(QtCore.Qt.AlignCenter)
        self.label_steering.setObjectName("label_steering")
        self.lcd_steering = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcd_steering.setGeometry(QtCore.QRect(70, 110, 241, 131))
        self.lcd_steering.setObjectName("lcd_steering")
        
        self.label_speed = QtWidgets.QLabel(self.centralwidget)
        self.label_speed.setGeometry(QtCore.QRect(100, 300, 181, 61))
        font = QtGui.QFont()
        font.setPointSize(27)
        font.setBold(True)
        font.setWeight(75)
        self.label_speed.setFont(font)
        self.label_speed.setAlignment(QtCore.Qt.AlignCenter)
        self.label_speed.setObjectName("label_speed")
        self.lcdspeed = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcdspeed.setGeometry(QtCore.QRect(70, 370, 241, 131))
        self.lcdspeed.setObjectName("lcdspeed")
        # -------------------------- Push Button PS ------------------------------------
        self.button_ps = QtWidgets.QPushButton(self.centralwidget)
        self.button_ps.setGeometry(QtCore.QRect(480, 290, 241, 71))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.button_ps.setFont(font)
        self.button_ps.setObjectName("button_ps")
        self.button_ps.clicked.connect(self.mode4)
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(450, 200, 290, 80))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_mode = QtWidgets.QLabel(self.horizontalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(18)
        self.label_mode.setFont(font)
        self.label_mode.setAlignment(QtCore.Qt.AlignCenter)
        self.label_mode.setObjectName("label_mode")
        self.horizontalLayout.addWidget(self.label_mode)

        # # -------------------------- Push Button Dockig ------------------------------------
        # self.button_doc = QtWidgets.QPushButton(self.centralwidget)
        # self.button_doc.setGeometry(QtCore.QRect(480, 290, 241, 71))
        # font = QtGui.QFont()
        # font.setPointSize(14)
        # self.button_doc.setFont(font)
        # self.button_doc.setObjectName("button_doc")
        # self.button_doc.clicked.connect(self.mode4)
        # self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        # self.horizontalLayoutWidget.setGeometry(QtCore.QRect(450, 200, 290, 80))
        # self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        # self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        # self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        # self.horizontalLayout.setObjectName("horizontalLayout")
        # self.label_mode = QtWidgets.QLabel(self.horizontalLayoutWidget)
        # font = QtGui.QFont()
        # font.setPointSize(18)
        # self.label_mode.setFont(font)
        # self.label_mode.setAlignment(QtCore.Qt.AlignCenter)
        # self.label_mode.setObjectName("label_mode")
        # self.horizontalLayout.addWidget(self.label_mode)
        
        self.chang_mode = QtWidgets.QLabel(self.horizontalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(19)
        font.setBold(True)
        font.setWeight(75)
        self.chang_mode.setFont(font)
        self.chang_mode.setAlignment(QtCore.Qt.AlignCenter)
        self.chang_mode.setObjectName("chang_mode")
        self.horizontalLayout.addWidget(self.chang_mode)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1000, 25))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def callback_controlAlgorithm(self, msg):
        self.cs_lamp = msg.pose.covariance[3]

    def callback_data_conv(self, msg):
        self.steer_now = msg.pose.covariance[0]
        self.V_now = msg.pose.covariance[6]

        if self.mode_auto == 4: self.mode4()
        elif self.mode_auto == 0: self.mode0()
        self.display_dash()
    # ------------------------ Control Mode --------------------------------------
    def mode0(self):       #MODE0
        self.mode_auto = 0
        self.mode_auto_bef = self.mode_auto
        self.chang_mode.setText("MANUAL")

    def mode4(self):  #MODE4        
        # check state before
        # print("masuk point")
        if self.mode_auto_bef == 0: 
            self.mode_auto = 4
            self.mode_auto_bef = self.mode_auto
            self.cs_lamp = 1
        # check whether the command has been complete or not
        if self.cs_lamp == 1:
            self.chang_mode.setText("AUTONOMOUS")            
        else:
            # bring back to stand by mode
            self.mode_auto = 0
            self.mode_auto_bef = self.mode_auto
            self.chang_mode.setText("MANUAL")
        # publish message
        self.idc_lamp()  

    def idc_lamp(self):            
        self.msg.pose.covariance[0] = self.mode_auto
        self.pub.publish(self.msg)

    def display_dash(self):
        # self.callback_data_collector()
        self.lcdspeed.display(str(round(self.V_now*3.6,1)))        #Speed
        self.lcd_steering.display(str(round(self.steer_now,1)))  

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_steering.setText(_translate("MainWindow", "STEERING"))
        self.label_speed.setText(_translate("MainWindow", "SPEED"))
        self.button_ps.setText(_translate("MainWindow", "POINT STABILIZATION"))
        self.label_mode.setText(_translate("MainWindow", "MODE :"))
        self.chang_mode.setText(_translate("MainWindow", "AUTONOMOUS"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

