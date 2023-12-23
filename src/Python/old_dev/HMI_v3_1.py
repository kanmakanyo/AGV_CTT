# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'HMI_v3.ui'
#
# Created by: GAN


import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QWidget, QLabel, QLineEdit
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtCore import QSize 

import sys
import os
import qdarkstyle

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1280, 720)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        app.setStyleSheet(qdarkstyle.load_stylesheet_pyside2())

        # Publish in ROS
        self.pub = rospy.Publisher("pyqt_topic", Twist, queue_size=10)
        rospy.init_node('pyqt_gui')                       
        # Subscribe in ROS
        self.sub = rospy.Subscriber('/PLC_data', Twist, self.sub_plc)
        
        self.current_value_plus = 2.5
        self.current_value_min = 2.5
        self.current_value_plus2 = 0
        self.current_value_min2 = 0
        #self.coba_inject = 0
        self.angle2volt = 2.68
        

        self.button_steer = QtWidgets.QPushButton(self.centralwidget)
        self.button_steer.clicked.connect(self.start)
        self.button_steer.setGeometry(QtCore.QRect(100, 70, 161, 61))
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.button_steer.setFont(font)
        self.button_steer.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.button_steer.setIconSize(QtCore.QSize(16, 16))
        self.button_steer.setObjectName("button_steer")
        self.slider_steer = QtWidgets.QSlider(self.centralwidget)
        self.slider_steer.setGeometry(QtCore.QRect(120, 150, 491, 121))
        self.slider_steer.setAutoFillBackground(False)
        self.slider_steer.setMaximum(5)
        self.slider_steer.setProperty("value", 2.5)
        self.slider_steer.setSliderPosition(2.5)
        self.slider_steer.setTracking(True)
        self.slider_steer.setOrientation(QtCore.Qt.Horizontal)
        self.slider_steer.setInvertedAppearance(False)
        self.slider_steer.setInvertedControls(False)
        self.slider_steer.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.slider_steer.setTickInterval(20)
        self.slider_steer.setObjectName("slider_steer")
        self.slider_steer.setObjectName("slider_steer")
        #self.slider_steer.valueChanged.connect(self.changeValue)
        #self.slider_steer.valueChanged.connect(self.publish_topic) # publish when slider steer
        
        self.steer_plus = QtWidgets.QPushButton(self.centralwidget)
        self.steer_plus.clicked.connect(self.changeValue_plus)          # plus clicked steer
        # self.steer_plus.clicked.connect(self.sub_plc)
        self.steer_plus.clicked.connect(self.publish_topic)
        self.steer_plus.clicked.connect(self.inject_sudut)
        self.steer_plus.setGeometry(QtCore.QRect(630, 180, 51, 51))
        font = QtGui.QFont()
        font.setPointSize(30)
        self.steer_plus.setFont(font)
        self.steer_plus.setObjectName("steerplus")
        self.steer_minus = QtWidgets.QPushButton(self.centralwidget)
        self.steer_minus.clicked.connect(self.changeValue_min)          # min clicked steer
        # self.steer_minus.clicked.connect(self.sub_plc)
        self.steer_minus.clicked.connect(self.publish_topic)
        self.steer_minus.clicked.connect(self.inject_sudut)
        self.steer_minus.setGeometry(QtCore.QRect(50, 180, 51, 51))
        font = QtGui.QFont()
        font.setPointSize(30)
        self.steer_minus.setFont(font)
        self.steer_minus.setObjectName("steer_minus")

        self.lcd_set_steer = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcd_set_steer.setGeometry(QtCore.QRect(50, 330, 231, 111))
        self.lcd_set_steer.setObjectName("lcd_set_steer")
        self.lcd_set_steer.display(str(0))
        self.lcd_feed_steer = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcd_feed_steer.setGeometry(QtCore.QRect(450, 330, 231, 111))
        self.lcd_feed_steer.setObjectName("lcd_feed_steer")

        self.button_throttle = QtWidgets.QPushButton(self.centralwidget)
        self.button_throttle.setGeometry(QtCore.QRect(740, 70, 161, 61))
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.button_throttle.setFont(font)
        self.button_throttle.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.button_throttle.setObjectName("button_throttle")
        self.slider_throttle = QtWidgets.QSlider(self.centralwidget)
        self.slider_throttle.setGeometry(QtCore.QRect(748, 250, 160, 311))
        self.slider_throttle.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.slider_throttle.setMaximum(24)
        self.slider_throttle.setOrientation(QtCore.Qt.Vertical)
        self.slider_throttle.setObjectName("slider_throttle")
        #self.slider_throttle.valueChanged.connect(self.changeValue2)
        #self.slider_throttle.valueChanged.connect(self.publish_topic) # publish when slider
        self.lcd_set_throttle = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcd_set_throttle.setGeometry(QtCore.QRect(960, 240, 231, 111))
        self.lcd_set_throttle.setObjectName("lcd_set_throttle")
        self.lcd_feed_throttle = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcd_feed_throttle.setGeometry(QtCore.QRect(960, 460, 231, 111))
        self.lcd_feed_throttle.setObjectName("lcd_feed_throttle")
        
        self.label_steer = QtWidgets.QLabel(self.centralwidget)
        self.label_steer.setGeometry(QtCore.QRect(58, 290, 221, 21))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_steer.setFont(font)
        self.label_steer.setAlignment(QtCore.Qt.AlignCenter)
        self.label_steer.setObjectName("label_steer")
        self.label_feed_steer = QtWidgets.QLabel(self.centralwidget)
        self.label_feed_steer.setGeometry(QtCore.QRect(454, 290, 231, 21))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_feed_steer.setFont(font)
        self.label_feed_steer.setAlignment(QtCore.Qt.AlignCenter)
        self.label_feed_steer.setObjectName("label_feed_steer")
        self.label_throttle = QtWidgets.QLabel(self.centralwidget)
        self.label_throttle.setGeometry(QtCore.QRect(960, 203, 231, 21))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_throttle.setFont(font)
        self.label_throttle.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_throttle.setAlignment(QtCore.Qt.AlignCenter)
        self.label_throttle.setObjectName("label_throttle")
        self.label_feed_throttle = QtWidgets.QLabel(self.centralwidget)
        self.label_feed_throttle.setGeometry(QtCore.QRect(960, 413, 231, 21))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_feed_throttle.setFont(font)
        self.label_feed_throttle.setAlignment(QtCore.Qt.AlignCenter)
        self.label_feed_throttle.setObjectName("label_feed_throttle")
        
        self.throttle_plus = QtWidgets.QPushButton(self.centralwidget)
        self.throttle_plus.clicked.connect(self.changeValue_plus2)
        # self.throttle_plus.clicked.connect(self.sub_plc)
        self.throttle_plus.clicked.connect(self.publish_topic)
        # self.throttle_plus.clicked.connect(self.inject_sudut) 
        self.throttle_plus.setGeometry(QtCore.QRect(800, 180, 51, 51))
        font = QtGui.QFont()
        font.setPointSize(30)
        self.throttle_plus.setFont(font)
        self.throttle_plus.setObjectName("throttle_plus")
        self.throttle_minus = QtWidgets.QPushButton(self.centralwidget)
        self.throttle_minus.clicked.connect(self.changeValue_min2)
        # self.throttle_minus.clicked.connect(self.sub_plc)
        self.throttle_minus.clicked.connect(self.publish_topic)
        # self.throttle_minus.clicked.connect(self.inject_sudut) 
        self.throttle_minus.setGeometry(QtCore.QRect(800, 580, 51, 51))
        font = QtGui.QFont()
        font.setPointSize(30)
        self.throttle_minus.setFont(font)
        self.throttle_minus.setObjectName("throttle_minus")
        
        self.inject_steer = QtWidgets.QLineEdit(self.centralwidget)
        self.inject_steer.setGeometry(QtCore.QRect(430, 470, 171, 51))
        font = QtGui.QFont()
        font.setPointSize(29)
        self.inject_steer.setFont(font)
        self.inject_steer.setText("")
        self.inject_steer.setObjectName("inject_steer")
        self.inject_steerOK = QtWidgets.QPushButton(self.centralwidget)
        self.inject_steerOK.setGeometry(QtCore.QRect(630, 470, 99, 51))
        self.inject_steerOK.clicked.connect(self.inject_sudut)
        self.inject_steerOK.clicked.connect(self.publish_topic)
        # self.inject_steerOK.clicked.connect(self.sub_plc)
        
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.inject_steerOK.setFont(font)
        self.inject_steerOK.setObjectName("inject_steerOK")


        self.inject_throttle = QtWidgets.QLineEdit(self.centralwidget)
        self.inject_throttle.setGeometry(QtCore.QRect(940, 590, 171, 51))
        font = QtGui.QFont()
        font.setPointSize(29)
        self.inject_throttle.setFont(font)
        self.inject_throttle.setText("")
        self.inject_throttle.setObjectName("inject_throttle")
        self.inject_throttleOK = QtWidgets.QPushButton(self.centralwidget)
        self.inject_throttleOK.setGeometry(QtCore.QRect(1140, 589, 99, 51))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.inject_throttleOK.setFont(font)
        self.inject_throttleOK.setObjectName("inject_throttleOK")

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

        self.is_running = False

    # Start Tes Throttle or Steer
    def start(self):
        if not self.is_running:
            self.is_running = True
            self.lcd_set_steer.display(str(self.current_value_plus))
            self.lcd_set_steer.display(str(self.current_value_min))
            self.lcd_set_throttle.display(str(self.current_value_plus2))
            self.lcd_set_throttle.display(str(self.current_value_min2))
        else:    
            print("Already running")

    # Function Publish
    def publish_topic(self):
        # self.pub.publish(str(self.current_value))
        self.msg = Twist()        
        self.msg.linear.x = self.current_value_plus      # Data from Steer
        self.msg.linear.x = self.current_value_min
        self.msg.linear.y = self.current_value_plus2     # Data from Throttle
        self.msg.linear.y = self.current_value_min2
        self.msg.linear.z = self.coba_inject             # Data from inject sudut 
        self.pub.publish(self.msg)
    
    # Function Subscribe (control part)
    def sub_plc(self, msg2):                  
        # self.data_feed_steer = 2.68
        self.data_feed_steer = msg2.angular.x
        self.data_feed_throttle = msg2.angular.y
        if self.data_feed_steer < (float(self.angle2volt) + float(0.02)) and self.data_feed_steer > (float(self.angle2volt)-float(0.02)):
            self.coba_inject = float(2.6) # Lurus
            print('Inject: ' + str(self.coba_inject))
        elif self.data_feed_steer > float(self.angle2volt):
            self.coba_inject = float(3.5) # Belok ke kanan
            print('Inject: ' + str(self.coba_inject))
        elif self.data_feed_steer < float(self.angle2volt):
            self.coba_inject = float(2.3) # Belok Ke kiri
            print('Inject: ' + str(self.coba_inject))

        print('Feedback Steer: ' + str(self.data_feed_steer))
        # print('Tegangan : ' + str(self.angle2volt)) 
        # print(msg2.angular.x)

        # Conversi feedback volt to angle for display
        self.a = 0.013#0.006862779022261
        self.c = 2.6603#2.67369356571782
        self.feedSteer_volt2angle = (self.data_feed_steer-self.c)/self.a
        # self.feedSteer_volt2angle = (((self.data_feed_steer - float(self.c)) / float(self.a))) + float(9)                
        self.lcd_feed_steer.display((self.feedSteer_volt2angle))

        self.lcd_feed_throttle.display((self.data_feed_throttle))       
        self.publish_topic()
        # self.msg.linear.x = self.current_value_plus      # Data from Steer
        # self.msg.linear.x = self.current_value_min
        # self.msg.linear.z = self.coba_inject             # Data from inject sudut 
        # self.pub.publish(self.msg)
        # print(self.msg.linear.z)
    # LCD Change value - SLider
    # Steering
    def changeValue_plus(self):       
        if self.is_running:                        
            self.current_value_plus += 0.1
            self.lcd_set_steer.display((self.current_value_plus))
            self.current_value_min = self.current_value_plus
        else:
            print("Not Running")
    def changeValue_min(self):
        if self.is_running:                        
            self.current_value_min -= 0.1
            self.lcd_set_steer.display((self.current_value_min))
            self.current_value_plus = self.current_value_min
        else:
            print("Not Running")
    # Throttle
    def changeValue_plus2(self):       
        if self.is_running:                        
            self.current_value_plus2 += 0.1
            self.lcd_set_throttle.display((self.current_value_plus2))
            self.current_value_min2 = self.current_value_plus2
        else:
            print("Not Running")
    def changeValue_min2(self):
        if self.is_running:                        
            self.current_value_min2 -= 0.1
            self.lcd_set_throttle.display((self.current_value_min2))
            self.current_value_plus2 = self.current_value_min2
        else:
            print("Not Running")

    # Steering : Inject sudut using input textbox inject
    def inject_sudut(self):
        # self.data_injectSudut = self.inject_steer.text()
        self.data_injectSudut = self.current_value_min
        # Sudut 50 - 130 ----> Tegangan 2.41 - 2.96
        self.a = 0.013#0.006862779022261
        self.c = 2.6603#2.67369356571782
        # self.data_injectSudut = self.current_value_min
        self.angle2volt = float(self.a) * float(self.data_injectSudut) + float(self.c)
        # print(self.angle2volt)
        # Display set steer change
        self.lcd_set_steer.display(self.data_injectSudut)

    # def inject_throttle(self):
    #     self.data_injectThrottle = self.current_value_min2
    #     self.publish_topic()

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.button_steer.setText(_translate("MainWindow", "STEERING"))
        self.button_throttle.setText(_translate("MainWindow", "THROTTLE"))
        self.label_steer.setText(_translate("MainWindow", "STEERING SET"))
        self.label_feed_steer.setText(_translate("MainWindow", "FEEDBACK STEERING"))
        self.label_throttle.setText(_translate("MainWindow", "THROTTLE SET"))
        self.label_feed_throttle.setText(_translate("MainWindow", "FEEDBACK THROTTLE"))
        self.steer_plus.setText(_translate("MainWindow", "+"))
        self.steer_minus.setText(_translate("MainWindow", "-"))
        self.throttle_plus.setText(_translate("MainWindow", "+"))
        self.throttle_minus.setText(_translate("MainWindow", "-"))
        self.inject_steerOK.setText(_translate("MainWindow", "INJECT"))
        self.inject_throttleOK.setText(_translate("MainWindow", "INJECT"))

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

rospy.spin()


