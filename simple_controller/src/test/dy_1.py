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

'''...................................Declare Constant.................................................'''
a_max =1
a_min =-1
nor_max = 20.0
nor_min = 1.0
theta_g_max = 4.0
theta_g_min = -4.0
dg_max = 20.0
dg_min = 0.0


laser = []
x1 = 0.0
y1 = 0.0
kw = 0.5
kv = 0.1
dx = 0.0
dy = 0.0
e = 0.0
theta1 = 0.0
px1 = 0.0
py1 = 0.0
path_dist1 = 0.0
path_dist = 0.0
#z_model = load_model('dy_angular5_lsrange20m_o6.h5')
z_model = load_model('dy_angular6_lsrange20m_o6.h5')
x_model = load_model('dy_linear5_lsrange20_o6.h5')
w_model = load_model('dy_ang50.h5')
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

sub = rospy.Subscriber("robot1/odom", Odometry, newOdom1)
sub1 = rospy.Subscriber("robot1/base_scan", LaserScan, newOdom)
pub = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=1)
counter = 0
r = rospy.Rate(10)
r1 = rospy.Rate(1)
speed = Twist()
goal = Point()
goal.x = 7.88 #7.88
goal.y = 4.69  # 4.58 ,-5.66

#px1 = 7.0
#py1 = -0.056
while not rospy.is_shutdown():
    inc_x = goal.x - x1
    inc_y = goal.y - y1
    angle_to_goal = atan2(inc_y, inc_x)
    theta_g = angle_to_goal
    dg = sqrt(inc_x * inc_x + inc_y * inc_y)

    theta_g_n = (2 * ((theta_g - theta_g_min) / (theta_g_max - theta_g_min)) - 1)
    dg_n = (2 * ((dg - dg_min) / (dg_max - dg_min)) - 1)

    theta_dg = list([theta_g_n, dg_n])
    z_laser = laser
    z_laser = np.array(z_laser)
    z_laser = z_laser.astype('float32')
    z_laser[z_laser == inf] = 20
    x_value_nor = (2 * ((z_laser - nor_min) / (nor_max - nor_min)) - 1)
    # x_value_nor = [((z_laser - nor_min) / (nor_max - nor_min))]
    theta_dg = np.array(theta_dg)

    x_value_nor = np.array(x_value_nor)
   


    if counter > 2:

        theta_dg = theta_dg.reshape(1, len(theta_dg))
        x_value_nor = x_value_nor.reshape((1, 1, len(x_value_nor)))
        inc_px1 = x1 - px1
        inc_py1 = y1 - py1
        path_dist1 = sqrt(inc_px1 * inc_px1 + inc_py1 * inc_py1)
        #path_dist = path_dist + path_dist1
        print("dg", dg)
        px1 = x1
        py1 = y1
        print(x_value_nor)
        #print('path_dist = ', path_dist)
        angular_velocity = z_model.predict([x_value_nor, x_value_nor, theta_dg, theta_dg])
        linear_velocity = x_model.predict([x_value_nor, theta_dg, theta_dg])
        x_pred = (angular_velocity * a_max + (a_min * (1 - angular_velocity)))
        if linear_velocity > 1:
            linear_velocity = 0.8
        if angular_velocity > 1:
            angular_velocity = 1
        print('angular_velocity', angular_velocity)
        print('linear_velocity', linear_velocity)
        #print('Distance_to_goal =', dg)
        #speed.angular.z = angular_velocity
        #speed.linear.x = linear_velocity
        pub.publish(speed)
        if dg <= 2.0:
            goal.x = -8.49
            goal.y = -0.203



    r.sleep()
    counter = counter + 1
