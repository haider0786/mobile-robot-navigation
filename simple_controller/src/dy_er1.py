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
import time

z_model = load_model('/home/robita/PycharmProjects/classification/merge_A.h5')
x_model = load_model('/home/robita/PycharmProjects/classification/new_merge_L1.h5')
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
theta1 = 0.0
## e_delay (hz)
e_delay = 0.1
flag_c = 0
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
def find_Index(min_val, test_list):
    res = []
    for idx in range(0, len(test_list)):
        if test_list[idx] == min_val and test_list[idx] <= (min_val+1):
            res.append(idx)
    return(res)
def ret_angular(value_laser):
    rno_element = 0
    lno_element = 0
    min_v = min(value_laser)
    index_list = find_Index(min_v, value_laser)
    min_Index = min(index_list)
    max_Index = max(index_list)
    if(min_Index == 0):
       print("rotate left")
       return 0.1
    else:
        rno_element = min_Index-0
    if (max_Index == 1018):
       print("rotate right1")
       return -0.1
    else:
        lno_element = 1018 - max_Index
    if rno_element >= lno_element:
       print("rno_element",rno_element)
       print("lno_element", lno_element)
       print("rotate_right2")
       return -0.1
    else:
       print("rotate_left3 ")
       print("rno_element", rno_element)
       print("lno_element", lno_element)
       return 0.1



sub = rospy.Subscriber("robot1/base_scan", LaserScan, newOdom)
sub1 = rospy.Subscriber("robot1/odom", Odometry, newOdom1)
sub2 = rospy.Subscriber("robot2/odom", Odometry, newOdom2)
pub = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=1)
r = rospy.Rate(freq)
delay = rospy.Rate(e_delay)
goal = Point()
speed = Twist()
goal_x = [8.48, -8.48]
goal_y = [0.08, 0.08]
j = 0
check = 0
flag_el = 0
linear_velocity = 1
pre_linear_velocity = 1
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
                Input_w1 = np.array(laser)
                Input_w1[Input_w1 == inf] = 20
                min_val = min(Input_w1)
                print("minimum_distance", min_val)
                Input_w = norm_laser(Input_w)
                if min_val <= 0.5:
                    speed.angular.z = 0.0
                    speed.linear.x = 0.0
                    pub.publish(speed)
                    print("Emergency_stop")
                    flag_c = 1
                    flag_el = 1+flag_el
                    time.sleep(5)
                    if flag_el == 3:
                       e_angular = ret_angular(laser)
                       speed.linear.x = -0.3
                       pub.publish(speed)
                       time.sleep(5)
                       speed.angular.z = e_angular
                       speed.linear.x = 0
                       pub.publish(speed)
                       time.sleep(5)
                       flag_el = 0

                Input_w_3d = Input_w.reshape((1, time_step, 1020))
                Input_pos_3d = Input_pos.reshape(1, 2)

                angular_velocity = z_model.predict([Input_w_3d, Input_pos_3d])
                linear_velocity = x_model.predict([Input_w_3d, Input_pos_3d])
                disR_x = x2-x1
                disR_y = y2-y1
                dist_bt_robots = sqrt(disR_x*disR_x + disR_y*disR_y)
                temp_ang_dg = [theta_g, dg, x1, y1, goal.x, goal.y, dist_bt_robots, theta1]
                angl_dist.append(temp_ang_dg)
                # if linear_velocity > 0.5:
                # linear_velocity = 0.5
                print("linear_velocity", linear_velocity)
                print("angular_velocity", angular_velocity)
                if pre_linear_velocity <= 0.01 and linear_velocity >= 0.5:
                    linear_velocity = 0.2
                print("linear_velocity_updated", linear_velocity)   
                if pre_linear_velocity == 0.2 and linear_velocity >= 0.5:
                    linear_velocity = 0.3
                print("linear_velocity_updated", linear_velocity)
                print("dg", dg)
                if linear_velocity <= 0.01 and angular_velocity <= 0.01:
                    check = check+1
                    print("check", check)
                    if check == 20:
                       e_angular = ret_angular(laser)
                       speed.angular.z = e_angular
                       pub.publish(speed)
                       time.sleep(5)
                       check = 0
                else:
                    check = 0
                if flag_c == 0:
                    speed.angular.z = angular_velocity
                    speed.linear.x = linear_velocity
                    pub.publish(speed)
                if dg <= 1:
                    if j == 0:
                        j = j + 1
                    else:
                        j = 0
                counter_in = 0
                Input_w = []
                Input_pos = []
        counter_o = counter_o + 1
        r.sleep()
        flag_c = 0
        pre_linear_velocity = linear_velocity



    except KeyboardInterrupt:
        break

angl_dist = np.array(angl_dist)
angl_dist1 = pd.DataFrame(angl_dist)
#angl_dist1.to_csv('/home/haider/result_sr2/multi_step/Dynamic/mdcr110_new.csv')
