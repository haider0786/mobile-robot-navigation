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
from keras.models import load_model
nor_max = 30.0
nor_min = 0.0
laser = []
x1 = 0.0
y1 = 0.0
kw = 0.5
kv = 0.1
dx = 0.0
dy = 0.0
e = 0.0
px1 = 0.0
py1 = 0.0
path_dist1 = 0.0
path_dist = 0.0
x_model = load_model('vel_w.h5')



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

rospy.init_node("LSTM_controller")

sub = rospy.Subscriber("robot1/odom", Odometry, newOdom1)
sub1 = rospy.Subscriber("robot1/base_scan", LaserScan, newOdom)
pub = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=10)
counter = 0
r = rospy.Rate(0.5)
r1 = rospy.Rate(0.5)
speed = Twist()
goal = Point()
goal.x = -13.0
goal.y = 0.465
px1 = 7.0
py1 = -0.056
while not rospy.is_shutdown():

    inc_x = goal.x - x1
    inc_y = goal.y - y1
    angle_to_goal = atan2(inc_y, inc_x)

    dg = sqrt(inc_x * inc_x + inc_y * inc_y)
    z_laser = list(laser)
    z_laser = np.array(z_laser)
    z_laser = z_laser.astype('float32')
    x_value_nor = (2 * ((z_laser - nor_min) / (nor_max - nor_min)) - 1)
    #x_value_nor = [((z_laser - nor_min) / (nor_max - nor_min))]
    #print('laser=',z_laser.shape)
    #print('x,y=',x1, y1)
    #print('counter=', counter)
    #print('x_value_nor=',x_value_nor)
    x_value_nor = np.array(x_value_nor)

    if counter >10:
        inc_px1 = x1 - px1
        inc_py1 = y1 - py1
        path_dist1 = sqrt(inc_px1 * inc_px1 + inc_py1 * inc_py1)
        path_dist = path_dist + path_dist1
        px1 = x1
        py1 = y1
        print('Path_dist = ', path_dist)
        x_value_nor = x_value_nor.reshape((1, 1, len(z_laser)))
        temp_results = x_model.predict((x_value_nor))
        x_pred = (temp_results * nor_max + (nor_min * (1 - temp_results)))
        print('Angular_velocity',temp_results)
	temp_results[np.isnan(temp_results)] = 0
	print('temp_results=', temp_results) 
        speed.angular.z = temp_results
        speed.linear.x = 0.8
        pub.publish(speed)
        if dg <= 3:
            speed.angular.z = 0.0
            speed.linear.x = 0.0
            pub.publish(speed)
            break


    r.sleep()
    counter = counter +1
while not rospy.is_shutdown():
    inc_px1 = x1 - px1
    inc_py1 = y1 - py1
    path_dist1 = sqrt(inc_px1 * inc_px1 + inc_py1 * inc_py1)
    path_dist = path_dist + path_dist1
    px1 = x1
    py1 = y1
    print('path_dist = ', path_dist)

    inc_x = goal.x - x1
    inc_y = goal.y - y1
    angle_to_goal = atan2(inc_y, inc_x)
    theta_g = atan2(sin(angle_to_goal - theta1), cos(angle_to_goal - theta1))
    dg = sqrt(inc_x * inc_x + inc_y * inc_y)
    if abs(theta_g) > 0.01:
        speed.angular.z = (kw * theta_g)
        speed.linear.x = 0.0
    else:
        speed.linear.x = 0.8
        speed.angular.z = 0.0
    pub.publish(speed)

    if dg <= 1.5:
        speed.angular.z = 0.0
        speed.linear.x = 0.0
        pub.publish(speed)
        break
    print("Distance_to_goal=", dg)
    r1.sleep()

