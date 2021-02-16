#! /usr/bin/env python
import rospkg
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from math import sin
from math import cos
from math import sqrt
import math
import numpy as np
import pandas as pd
from pandas import read_csv
from keras.models import load_model
obstacle_x = -2.0
obstacle_y = 2.0
obstacle_x1 = -2.0
obstacle_y1 = 1.0
obstacle_x2 = -2.0
obstacle_y2 = 0.0
obstacle_x3 = -2.0
obstacle_y3 = -1.0
obstacle_x4 = -2.0
obstacle_y4 = -2.0
x1 = 0.0
y1 = 0.0
dg = 2.0
v = 0.0
w = 0.0
theta1 = 0.0
value1 = []
value1_angle = []
w_model = load_model('angle_d4_w.h5')
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
   global v
   global w, theta_o,theta_o1,theta_o2,theta_o3,theta_o4, theta_g, do, do1, do2, do3, do4, dg

   v = msg.linear.x
   w = msg.angular.z
rospy.init_node("speed_controller")

sub1 = rospy.Subscriber("robot1/odom", Odometry, newOdom1)
sub2 = rospy.Subscriber("robot1/cmd_vel", Twist, newOdom2)
pub = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=1)

r = rospy.Rate(10)
speed = Twist()
goal = Point()

goal.x = -11.0
goal.y = 0.44

i = 0   #'''intilization of counter'''
while not rospy.is_shutdown():
    if i > 5:
        '''goal'''
        inc_x = goal.x - x1
        inc_y = goal.y - y1
        angle_to_goal = atan2(inc_y, inc_x)
        theta_g = atan2(sin(angle_to_goal - theta1), cos(angle_to_goal - theta1))
        dg = sqrt(inc_x * inc_x + inc_y * inc_y)
        theta_g_max = 4.0
        theta_g_min = -4.0
        dg_max = 20.0
        dg_min = 1.0
        theta_g_n = (2*((theta_g-theta_g_min)/(theta_g_max-theta_g_min)) -1)
        dg_n = (2*((dg-dg_min)/(dg_max-dg_min))-1)
        '''obstale 0'''
        inc_ox = obstacle_x - x1
        inc_oy = obstacle_y - y1
        angle_to_ob = atan2(inc_oy, inc_ox)
        theta_o = atan2(sin(angle_to_ob - theta1), cos(angle_to_ob - theta1))
        do = sqrt(inc_ox * inc_ox + inc_oy * inc_oy)
        theta_o_max = 4.0
        theta_o_min = -4.0
        do_max = 11.0
        do_min = 2.0
        theta_o_n = (2 * ((theta_o - theta_o_min) / (theta_o_max - theta_o_min)) - 1)
        do_n = (2 * ((do - do_min) / (do_max - do_min)) - 1)

        '''obstale 1'''
        inc_ox1 = obstacle_x1 - x1
        inc_oy1 = obstacle_y1 - y1
        angle_to_ob1 = atan2(inc_oy1, inc_ox1)
        theta_o1 = atan2(sin(angle_to_ob1 - theta1), cos(angle_to_ob1 - theta1))
        # dx_o1 = obstacle_x1 - inc_ox1
        # dy_o1 = obstacle_y1 - inc_oy1
        do1 = sqrt(inc_ox1 * inc_ox1 + inc_oy1 * inc_oy1)
        theta_o1_max = 4.0
        theta_o1_min = -4.0
        do1_max = 11.0
        do1_min = 2.0
        theta_o1_n = (2 * ((theta_o1 - theta_o1_min) / (theta_o1_max - theta_o1_min)) - 1)
        do1_n = (2 * ((do1 - do1_min) / (do1_max - do1_min)) - 1)

        '''obstale 2'''
        inc_ox2 = obstacle_x2 - x1
        inc_oy2 = obstacle_y2 - y1
        angle_to_ob2 = atan2(inc_oy2, inc_ox2)
        theta_o2 = atan2(sin(angle_to_ob2 - theta1), cos(angle_to_ob2 - theta1))
        # dx_o2 = obstacle_x2 - inc_ox2
        # dy_o2 = obstacle_y2 - inc_oy2
        do2 = sqrt(inc_ox2 * inc_ox2 + inc_oy2 * inc_oy2)
        theta_o2_max = 4.0
        theta_o2_min = -4.0
        do2_max = 11.0
        do2_min = 2.0
        theta_o2_n = (2 * ((theta_o2 - theta_o2_min) / (theta_o2_max - theta_o2_min)) - 1)
        do2_n = (2 * ((do2 - do2_min) / (do2_max - do2_min)) - 1)
        '''obstale 3'''
        inc_ox3 = obstacle_x3 - x1
        inc_oy3 = obstacle_y3 - y1
        angle_to_ob3 = atan2(inc_oy3, inc_ox3)
        theta_o3 = atan2(sin(angle_to_ob3 - theta1), cos(angle_to_ob3 - theta1))
        # dx_o3 = obstacle_x3 - inc_ox3
        # dy_o3 = obstacle_y3 - inc_oy3
        do3 = sqrt(inc_ox3 * inc_ox3 + inc_oy3 * inc_oy3)
        theta_o3_max = 4.0
        theta_o3_min = -4.0
        do3_max = 11.0
        do3_min = 2.0
        theta_o3_n = (2 * ((theta_o3 - theta_o3_min) / (theta_o3_max - theta_o3_min)) - 1)
        do3_n = (2 * ((do3 - do3_min) / (do3_max - do3_min)) - 1)
        '''obstale 4'''
        inc_ox4 = obstacle_x4 - x1
        inc_oy4 = obstacle_y4 - y1
        angle_to_ob4 = atan2(inc_oy4, inc_ox4)
        theta_o4 = atan2(sin(angle_to_ob4 - theta1), cos(angle_to_ob4 - theta1))
        # dx_o4 = obstacle_x4 - inc_ox4
        # dy_o4 = obstacle_y4 - inc_oy4
        do4 = sqrt(inc_ox4 * inc_ox4 + inc_oy4 * inc_oy4)
        theta_o4_max = 4.0
        theta_o4_min = -4.0
        do4_max = 11.0
        do4_min = 2.0
        theta_o4_n = (2 * ((theta_o4 - theta_o4_min) / (theta_o4_max - theta_o4_min)) - 1)
        do4_n = (2 * ((do4 - do4_min) / (do4_max - do4_min)) - 1)

        value1_angle = list([theta_g_n]) + list([dg_n]) + list([theta_o_n]) + list([do_n]) + list([theta_o1_n]) + list(
            [do1_n]) + list([theta_o2_n]) + list([do2_n]) + list([theta_o3_n]) + list([do3_n]) + list([theta_o4_n]) + list(
            [do4_n])
        value1 = np.array(value1_angle)
        value2 = value1.reshape((1,1,12))
        temp_results1 = w_model.predict((value2))
        w_max = 1.0
        w_min = -1.0
        w = (((temp_results1+1)*(w_max-w_min))/2 + w_min)
        print('value_angle=', value1_angle)
        print('w=',w)
	print('temp_results1=', temp_results1)
        speed.angular.z = temp_results1
        speed.linear.x = 0.8
        pub.publish(speed)
        r.sleep()
        r.sleep()
    i=i+1













