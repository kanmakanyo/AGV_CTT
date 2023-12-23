#! /usr/bin/env python

import math
from math import sin, cos, pi
import time
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
# from marvelmind_nav.msg import hedge_pos_ang
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
from agv_ctt.msg import LocalizationControllerResultMessage0502
from agv_ctt.msg import OdometryMessage0105


rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("marvelmind_odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

def to_euler(x, y, z, w): 
    roll = np.arctan2(2.0 * (y*x + w*z), w**2 + x**2 - y**2 - z**2)
    pitch = np.arcsin(-2.0 * (x*z - w*y)/(w**2 + x**2 + y**2 + z**2))
    yaw = np.arctan2(2.0 * (w*z + x*y), w**2 - x**2 - y**2 + z**2)
    return roll,pitch,yaw

x = 0.0
y = 0.0
x_prev = 0.0
y_prev = 0.0
vx = 0.0
vy = 0.0

th = 0.0
th_prev = 0.0
vth = 0.0

x_gps = 0.0
y_gps = 0.0
th_gps = 0.0

time_prev = rospy.Time.now()


# def marvelmind(msg):
#     global x_gps, y_gps,th_gps

#     x_gps = msg.x_m
#     y_gps = msg.y_m
#     th_gps = np.radians(msg.angle - 180)
#     # print(x_gps,y_gps,th_gps)

def callback_lidar_odom(msg):
    global x_gps, y_gps,th_gps
    # global vx,vy,vth

    # time_now = rospy.Time.now()
    # dt = time_now-time_prev

    x_gps = float(msg.x_position)/1000
    y_gps = float(msg.y_position)/1000
    th_gps = float(np.radians(msg.heading))/1000

    # vx = -(x_odom-x_prev)/dt
    # vy = -(y_odom-y_prev)/dt
    # vth = -(yaw_odom-yaw_prev)/dt
    # print(x_gps,y_gps,th_gps)
    # x_prev = x_odom
    # y_prev = y_odom
    # yaw_prev = yaw_odom
    # time_prev = time_now

r = rospy.Rate(10)
pub_msg = PoseWithCovarianceStamped()
pub_msg.header.frame_id = 'odometry_publisher'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()

# rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amclcallback)
# rospy.Subscriber('/hedge_pos_ang', hedge_pos_ang, marvelmind)
# rospy.Subscriber('/localizationcontroller/out/localizationcontroller_result_message_0502', \
#     LocalizationControllerResultMessage0502, callback_lidar_pos)
rospy.Subscriber('/localizationcontroller/out/odometry_message_0105', \
    OdometryMessage0105, callback_lidar_odom)

i = 0

while not rospy.is_shutdown():
    time_now = rospy.Time.now()

    if i == 0:
        x = x_gps
        y = y_gps
        x_prev = x_gps
        y_prev = y_gps
        vx = 0
        vy = 0
        th = th_gps
        th_prev = th_gps
        vth = 0
    else:
        x = x_gps
        y = y_gps
        vx = abs(x-x_prev)/0.1
        vy = abs(y-y_prev)/0.1
        x_prev = x_gps
        y_prev = y_gps

        th = th_gps
        vth = abs(th-th_prev)/0.1
        th_prev = th_gps
        
    i += 1
    # print(x_gps,y_gps,vx,vy)
    # compute odometry in a typical way given the velocities of the robot
    # dt = (time_now - time_prev).to_sec()
    # delta_x = (vx * np.cos(th) - vy * np.sin(th)) * dt
    # delta_y = (vx * np.sin(th) + vy * np.cos(th)) * dt
    # delta_th = vth * dt

    # x = x + delta_x
    # y = y + delta_y
    # th = th + delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th_gps)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x_gps, y_gps, 0),
        odom_quat,
        time_now,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = time_now
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x_gps,y_gps,0), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    time_prev = time_now
    r.sleep()
