#! /usr/bin/env python
import rospkg
import rospy
# from tf2_py import *
# from ._tf2 import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from math import sin
from math import cos
from math import sqrt
import numpy as np
import pandas as pd
from pandas import read_csv
from keras.models import load_model
from numpy import inf
laser = []
x_r = 0.0
y_r = 0.0
length = 0.0
length1 = 0.0
length2 = 0.0
diff_length = 0.0
length_stored = []
x_i = -8.48
y_i = 0.08
kw = 0.5
kv = 0.1
dx = 0.0
dy = 0.0
dg = 2.0  # initialize to pass the first loop condition
e = 0.0
v = 0.0
w = 0.0


j = 0
i = 0
k = 0
x_in = -8.48
y_in = 0.08
vel_ix = 0.0
vel_iy = 0.0
vel_nx = 0.0
vel_ny = 0.0
xob_in = 0.0
yob_in = 0.0
velob_nx = 0.0
velob_ny = 0.0
velob_ix = 0.0
velob_iy = 0.0

x_ob = 0.0
y_ob = 0.0
an_ob = 0.0
dg_ob = 0.0

velwo_ix = 0.0
velwo_iy = 0.0
velwo_nx = 0.0
velwo_ny = 0.0
xwo_i = 0.0
ywo_i = 0.0
xw_o = 0.0
yw_o = 0.0
velw_o = 0.0
accw_o = 0.0
time_step = 3
Input_w = []
counter_in = 0
counter_o = 0
Input_pos = []
obs_inf = []

linearvelocity_stored = []
angularvelociy_stored = []
odom_stored = []

z_model = load_model('/home/robita/PycharmProjects/classification/srcrstvel_A3.h5')
x_model = load_model('/home/robita/PycharmProjects/classification/srcrstvel_L3.h5')
def newOdom(msg):
    global laser
    laser = msg.ranges

def newOdom1(msg):
    global x_r
    global y_r
    global theta_r
    global x_final1
    global y_final1
    x_r = msg.pose.pose.position.x
    y_r = msg.pose.pose.position.y
    rot_q1 = msg.pose.pose.orientation
    (roll1, pitch1, theta_r) = euler_from_quaternion([rot_q1.x, rot_q1.y, rot_q1.z, rot_q1.w])
def odomrobot2(msg):
     global xw_o
     global yw_o
     xw_o = msg.pose.pose.position.x
     yw_o = msg.pose.pose.position.y
     rot_q2 = msg.pose.pose.orientation
     (roll2, pitch2, thetaw_o) = euler_from_quaternion([rot_q2.x, rot_q2.y, rot_q2.z, rot_q2.w])

def newOdom2(msg):
    global v
    global w, theta_g, dg
    v = msg.linear.x
    w = msg.angular.z

def euclideanaOrientation(x_e, y_e):
    diff_x = x_r - x_e
    diff_y = y_r - y_e
    angle_to_robot = atan2(diff_y, diff_x)
    eucd = sqrt(diff_x * diff_x + diff_y * diff_y)
    return (angle_to_robot, eucd)

def velocity(x_i,y_i, x_n,y_n):
    vx = x_n-x_i
    vy = y_n-y_i
    vl = vx + vy
    return(vl, vx, vy)

def acceleration(vel_ix, vel_iy, vel_nx, vel_ny):
    ax = vel_nx - vel_ix
    ay = vel_ny - vel_iy
    a = ax + ay
    return(a)

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

def acceleration_norm(value):
    acc_max = 227.0
    acc_min = -227.0
    value_n = (2 * ((value - acc_min) / (acc_max - acc_min)) - 1)
    return value_n
def velocity_norm(value):
    velo_max = 10.0
    velo_min = -24.0
    value_n = (2 * ((value - velo_min) / (velo_max - velo_min)) - 1)
    return value_n
def xposobs_norm(value):
    xpos_max = 10.0
    xpos_min = -10.0
    value_n = (2 * ((value - xpos_min) / (xpos_max - xpos_min)) - 1)
    return value_n

def yposobs_norm(value):
    ypos_max = 11.0
    ypos_min = -11.0
    value_n = (2 * ((value - ypos_min) / (ypos_max - ypos_min)) - 1)
    return value_n

def oriobs_norm(value):
    ori_max = 6.4
    ori_min = 0.0
    value_n = (2 * ((value - ori_min) / (ori_max - ori_min)) - 1)
    return value_n
def distobs_norm(value):
    dist_max = 10.0
    dist_min = 0.0
    value_n = (2 * ((value - dist_min) / (dist_max - dist_min)) - 1)
    return value_n



rospy.init_node("data_collection")

sub = rospy.Subscriber("robot1/base_scan", LaserScan, newOdom)
sub1 = rospy.Subscriber("robot1/odom", Odometry, newOdom1)
sub2 = rospy.Subscriber("robot1/cmd_vel", Twist, newOdom2)
sub3 = rospy.Subscriber("robot2/odom", Odometry, odomrobot2)
pub = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=1)
r = rospy.Rate(10)
goal = Point()
speed = Twist()
goal_x = [8.48, -8.48]
goal_y = [0.08, 0.08]
goal.x = 0.0
goal.y = 0.0

def find_Index(min_val, test_list):
    res = []
    idx = 0
    for idx in range(0, len(test_list)):
        if test_list[idx] <= min_val:
            res.append(idx)
    return(res)
def ret_angular(value_laser):
    if min(value_laser) <= 10:
        min_v = min(value_laser)
        index_list = find_Index(min_v, value_laser)
    else:
        return (0, 0, 0, 0)
    min_Index = min(index_list)
    max_Index = max(index_list)
    if max_Index < int(len(value_laser)/2):
        print("angle with max_index",max_Index)
        angle_step = 6.28/len(value_laser)
        max_angle = angle_step*max_Index
        dg_o = value_laser[max_Index]
        x_o = value_laser[max_Index]*cos(max_angle)
        y_o = value_laser[max_Index]*sin(max_angle)
        return(max_angle, dg_o, x_o, y_o)
    else:
        if(min_Index > int(len(value_laser)/2)):
            print("angle with min_index", min_Index)
            angle_step = 6.28/(len(value_laser))
            min_angle = angle_step*min_Index
            dg_o = value_laser[min_Index]
            x_o = value_laser[min_Index] * cos(min_angle)
            y_o = value_laser[min_Index] * sin(min_angle)
            return(min_angle, dg_o, x_o, y_o)
        else:
            print("angle with min_index else", min_Index)
            angle_step = 6.28/(len(value_laser))
            min_angle = angle_step * min_Index
            dg_o = value_laser[min_Index]
            x_o = value_laser[min_Index] * cos(min_angle)
            y_o = value_laser[min_Index] * sin(min_angle)
            return (min_angle, dg_o, x_o, y_o)


while True:
    try:
        if counter_o > 2:
            goal.x = goal_x[j]
            goal.y = goal_y[j]
            inc_x = goal.x - x_r
            inc_y = goal.y - y_r
            theta_g = atan2(inc_y, inc_x)
            [vel, vel_nx, vel_ny] = velocity(x_in, y_in, x_r, y_r)
            vel = 10 * vel
            acc = 10 * acceleration(vel_ix, vel_iy, vel_nx, vel_ny)
            dg = sqrt(inc_x * inc_x + inc_y * inc_y)
            dg_n = norm_euclidean(dg)
            theta_g_n = norm_theta(theta_g)
            theta_dg = list([theta_g_n, dg_n])
            Input_pos = theta_dg
            value1 = list(laser)
            Input_w.append(value1)
            '''..............................Calculation with respect to the world frame.............................'''

            [velw_o, velwo_nx, velwo_ny] = velocity(xwo_i, ywo_i, xw_o, yw_o)
            velw_o = 10 * velw_o
            accw_o = 10 * acceleration(velwo_ix, velwo_iy, velwo_nx, velwo_ny)

            '''.......................Calculation with respect to LiDAR frame............................................'''

            [an_ob, dg_ob, x_ob, y_ob] = ret_angular(laser)

            [vel_ob, velob_nx, velob_ny] = velocity(xob_in, yob_in, x_ob, y_ob)

            acc_ob = 10 * acceleration(velob_ix, velob_iy, velob_nx, velob_ny)
            acc_ob_norm = acceleration_norm(acc_ob)
            vel_ob_norm = velocity_norm(vel_ob)
            an_ob_norm = oriobs_norm(an_ob)
            dg_ob_norm = distobs_norm(dg_ob)
            x_ob_norm = xposobs_norm(x_ob)
            y_ob_norm = yposobs_norm(y_ob)
            obs_inf_value = list([acc_ob_norm, vel_ob_norm,  dg_ob_norm, an_ob_norm, x_ob_norm, y_ob_norm])
            obs_inf.append(obs_inf_value)

            # print("dg=", dg)
            # print('theta_g', theta_g)
            # print("v=", v)
            # print("w=", w)
            # print("Goal_Number=", j)
            # print("velocity", vel)
            # print("acceleration", acc)
            # print("x_in, y_in, x_r, y_r", x_in, y_in, x_r, y_r)
            # print("goal.x=")
            # laser = list(laser)

            # value1 = list([x, y])
            # value2 = value1 + list(laser)
            # print('value4:',value2)
            counter_in = counter_in + 1
            print('counter_in', counter_in)
            if counter_in == time_step:
                Input_w = np.array(Input_w)
                Input_w[Input_w == inf] = 20
                Input_pos = np.array(Input_pos)
                Input_w = norm_laser(Input_w)
                Input_w_3d = Input_w.reshape((1, time_step, 1020))
                Input_pos_3d = Input_pos.reshape(1, 2)
                obs_inf_value = np.array(obs_inf)
                Input_obs_info = obs_inf_value.reshape((1, time_step, 6))
                angular_velocity = (z_model.predict([Input_w_3d, Input_obs_info, Input_pos_3d]))[0]
                linear_velocity = (x_model.predict([Input_w_3d, Input_obs_info, Input_pos_3d]))[0]
                print('linear_velocity_before', linear_velocity)
                if linear_velocity > 0.7:
                    linear_velocity = 0.7
                else:
                    if linear_velocity < 0:
                        linear_velocity = 0.0
                    else:
                        linear_velocity = linear_velocity[0]

                speed.angular.z = angular_velocity
                speed.linear.x = linear_velocity

                linearvelocity_stored.append(linear_velocity)
                angularvelociy_stored.append(angular_velocity)
                distr1r2 = sqrt(pow(xw_o - x_r, 2) + pow(yw_o - y_r, 2))

                xy_pos = [x_r, y_r, xw_o, yw_o, distr1r2]
                odom_stored.append(xy_pos)

                print('angular_velocity1', angular_velocity)
                print('linear_velocity1', linear_velocity)
                pub.publish(speed)

                "calculation of length travelled while moving towards the goal"

                length2 = sqrt(pow(x_r - x_i, 2)+pow(y_r - y_i, 2))
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
                obs_inf = []
        x_in = x_r
        y_in = y_r
        xob_in = x_ob
        yob_in = y_ob
        vel_ix = vel_nx
        vel_iy = vel_ny
        velob_ix = velob_nx
        velob_iy = velob_ny
        xwo_i = xw_o
        ywo_i = yw_o
        velwo_ix = velwo_nx
        velwo_iy = velwo_ny
        counter_o = counter_o + 1
        r.sleep()
    except KeyboardInterrupt:
        break
print("i am outside the break")
linearvelocity_stored = pd.DataFrame(linearvelocity_stored)
angularvelociy_stored = pd.DataFrame(angularvelociy_stored)
odom_stored = pd.DataFrame(odom_stored)
length_stored = pd.DataFrame(length_stored)
linearvelocity_stored.to_csv('/home/robita/simulation_data/data_3paper/LSTM_random_test/0.25hz/25hz_L_velocity20.csv')
angularvelociy_stored.to_csv('/home/robita/simulation_data/data_3paper/LSTM_random_test/0.25hz/25hz_A_velocity20.csv')
odom_stored.to_csv('/home/robita/simulation_data/data_3paper/LSTM_random_test/0.25hz/25hz_xy_pos20.csv')
length_stored.to_csv('/home/robita/simulation_data/data_3paper/LSTM_random_test/0.25hz/25hz_length20.csv')

