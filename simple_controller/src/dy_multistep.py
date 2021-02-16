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

z_model = load_model( '/home/robita/catkin_ws/src/simple_controller/launch/mlstm_A.h5')
x_model = load_model('/home/robita/catkin_ws/src/simple_controller/launch/mlstm_L.h5')
angl_dist = []
angl_dist1 = []
laser = []
Input_pos = []
x1 = 0.0
y1 = 0.0
x2 = 0.0
y2 = 0.0
value1 = []
Input_w = []
counter_in = 0
counter_o = 0
freq = 10
time_step = 3
theta1 =0.0

length = 0.0
length1 = 0.0
length2 = 0.0
diff_length = 0.0
length_stored = []
linearvelocity_stored = []
angularvelociy_stored = []
odom_stored = []
x_i = -8.48
y_i = 0.08


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

def newOdom2(msg):
    global x2
    global y2
    global theta2
    global x_final2
    global y_final2
    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y

def newOdom(msg):
    global laser
    laser = msg.ranges

rospy.init_node("speed_controller1")


'''............................................Normalization..................................................................'''

def norm_laser(value):
    laser_max = 20
    laser_min = 0
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
sub2 = rospy.Subscriber("robot2/odom", Odometry, newOdom2)
pub = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=1)
r = rospy.Rate(freq)
goal = Point()
speed = Twist()
goal_x = [8.48, -8.48]
goal_y = [0.08, 0.08]
j = 0
while not rospy.is_shutdown():
    try:

        if counter_o > 2:
            goal.x = goal_x[j]
            goal.y = goal_y[j]
            inc_x = goal.x - x1
            inc_y = goal.y - y1
            angle_to_goal = atan2(inc_y, inc_x)
            theta_g = angle_to_goal
            dg = sqrt(inc_x * inc_x + inc_y * inc_y)
            dg_n = norm_euclidean(dg)
            theta_g_n = norm_theta(theta_g)

            theta_dg = list([theta_g_n, dg_n])
            Input_pos = theta_dg
            value1 = list(laser)
            Input_w.append(value1)
            counter_in = counter_in + 1
            if counter_in == time_step:
                Input_w = np.array(Input_w)
                Input_w[Input_w == inf] = 20
                Input_pos = np.array(Input_pos)
                Input_w = norm_laser(Input_w)

                # print Input_w_norm.shape
                # print Input_pos_norm.shape
                #print Input_w
                #print Input_pos
                #print 'Input_pos_shape', Input_pos.shape
                #print 'Input_w_shape', Input_w.shape
                Input_w_3d = Input_w.reshape((1, time_step, 1020))
                Input_pos_3d = Input_pos.reshape(1, 2)
                #print 'Input_w_3d shape', Input_w_3d.shape
                #print 'Input_pos_3d shape', Input_pos_3d.shape
                #print Input_w_3d
                #print Input_pos_3d
                angular_velocity = z_model.predict([Input_w_3d, Input_pos_3d])[0]
                linear_velocity = x_model.predict([Input_w_3d, Input_pos_3d])[0]
                print('dg1', dg)
                print('theta_g1', theta_g)
                disR_x = x2-x1
                disR_y = y2-y1
                dist_bt_robots = sqrt(disR_x*disR_x + disR_y*disR_y)
                temp_ang_dg = [theta_g, dg, x1, y1, goal.x, goal.y, dist_bt_robots, theta1]
                angl_dist.append(temp_ang_dg)
                if linear_velocity > 0.7:
                    linear_velocity = 0.7
                speed.angular.z = angular_velocity
                speed.linear.x = linear_velocity
                print('angular_velocity1', angular_velocity)
                print('linear_velocity1', linear_velocity)

                linearvelocity_stored.append(linear_velocity)
                angularvelociy_stored.append(angular_velocity)
                speed.angular.z = angular_velocity
                speed.linear.x = linear_velocity

                pub.publish(speed)

                "calculation of length travelled while moving towards the goal"
                length2 = sqrt(pow(x1 - x_i, 2) + pow(y1 - y_i, 2))
                diff_length = abs(length2 - length1)
                length += diff_length
                length1 = length2
                length_stored.append(length)

                if dg <= 1.5:
                    if j == 0:
                        j = j + 1
                    else:
                        j = 0
                counter_in = 0
                Input_w = []
                Input_pos = []
        counter_o = counter_o + 1
        r.sleep()
    except KeyboardInterrupt:
        break



angl_dist = np.array(angl_dist)
angl_dist1 = pd.DataFrame(angl_dist)
linearvelocity_stored = pd.DataFrame(linearvelocity_stored)
angularvelociy_stored = pd.DataFrame(angularvelociy_stored)
length_stored = pd.DataFrame(length_stored)
# linearvelocity_stored.to_csv('/home/robita/simulation_data/data_2paper/static/LSTM_headon/L_velocity5.csv')
# angularvelociy_stored.to_csv('/home/robita/simulation_data/data_2paper/static/LSTM_headon/A_velocity5.csv')
# length_stored.to_csv('/home/robita/simulation_data/data_2paper/static/LSTM_headon/length5.csv')
# angl_dist1.to_csv('/home/robita/simulation_data/data_2paper/static/LSTM_headon/xy_pos5.csv')

