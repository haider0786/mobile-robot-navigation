#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from sklearn.preprocessing import MinMaxScaler
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import mean_squared_error
from math import atan2
from math import sin
from math import cos
from math import sqrt
from pandas import read_csv
from pandas import DataFrame
from pandas import concat
import pandas as pd
import numpy as np
from numpy import inf
from keras.models import load_model

z_model = load_model('dy_multistep.h5')
x_model = load_model('dy_multistep_L.h5')

laser = []
Input_pos = []
x1 = 0.0
y1 = 0.0
value1 = []
Input_w = []
counter_in = 0
counter_o = 0

def newOdom1(msg):
    global x1
    global y1
    global theta1
    global x_final1
    global y_final1
    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y
    rot_q1 = msg.pose.pose.orientation
    (roll1, pitch1, theta1) = euler_from_quaternion([rot_q1.x, rot_q1.y, rot_q1.z, rot_q1.w])


def newOdom(msg):
    global laser
    laser = msg.ranges
rospy.init_node("speed_controller")


'''............................................Normalization..................................................................'''

def norm_laser(value):
    laser_max = 20
    laser_min = 1
    value_n = (2 * ((value - laser_min) / (laser_max - laser_min)) - 1)
    return value_n

def norm_euclidean(value):
    dg_max = 20.0
    dg_min = 0.0
    value_n = (2 * ((value - dg_min) / (dg_max - dg_min)) - 1)
    return value_n

def norm_theta(value):
    theta_max = 4.0
    theta_min = -4.0
    value_n = (2 * ((value - theta_min) / (theta_max - theta_min)) - 1)
    return value_n
'''..........................................................................................................................'''


sub = rospy.Subscriber("robot1/base_scan", LaserScan, newOdom)
sub1 = rospy.Subscriber("robot1/odom", Odometry, newOdom1)
pub = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=1)
r = rospy.Rate(12)
goal = Point()
speed = Twist()
# goal.x = 7.88  # 7.88
# goal.y = 4.69  # 4.58 ,-5.66

while not rospy.is_shutdown():
    if counter_o > 2:
        inc_x = goal.x - x1
        inc_y = goal.y - y1
        angle_to_goal = atan2(inc_y, inc_x)
        theta_g = angle_to_goal
        dg = sqrt(inc_x * inc_x + inc_y * inc_y)
        dg_n = norm_euclidean(dg)
        theta_g_n = norm_theta(theta_g)

        theta_dg = list([theta_g_n, dg_n])
        Input_pos.append(theta_dg)
        value1 = list(laser)
        Input_w.append(value1)
        counter_in = counter_in + 1
        if counter_in == 5:
            Input_w = np.array(Input_w)
            Input_w[Input_w == inf] = 20
            Input_pos = np.array(Input_pos)
            Input_w = norm_laser(Input_w)


            # print Input_w_norm.shape
            # print Input_pos_norm.shape
            print Input_w
            print Input_pos
            print 'Input_pos_shape', Input_pos.shape
            print 'Input_w_shape', Input_w.shape
            Input_w_3d = Input_w.reshape((1, 5, 1020))
            Input_pos_3d = Input_pos.reshape((1, 5, 2))
            print 'Input_w_3d shape', Input_w_3d.shape
            print 'Input_pos_3d shape', Input_pos_3d.shape
            print Input_w_3d
            print Input_pos_3d
            angular_velocity = z_model.predict([Input_w_3d, Input_w_3d, Input_pos_3d])
            linear_velocity = x_model.predict([Input_w_3d, Input_w_3d, Input_pos_3d])
            print 'angular_velocity', angular_velocity
            print 'linear_velocity', linear_velocity
            print 'dg', dg
            speed.angular.z = angular_velocity
            speed.linear.x = linear_velocity
            pub.publish(speed)
            if dg <= 2.0:
                goal.x = -8.49
                goal.y = -0.203

            counter_in = 0
            Input_w = []
            Input_pos = []
    counter_o = counter_o+1
    r.sleep()
