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
nor_max = 50
nor_min = -50

v_model = load_model('x_model_linear.h5')
w_model = load_model('x_model_angle.h5')
theta1 = 0.0
laser = []
x1 = 0.0
y1 = 0.0
kw = 0.5
kv = 0.1
dx = 0.0
dy = 0.0
e = 0.0
v = 0.0
w = 0.0
obstacle_x = 3.0
obstacle_y = 5.0
value1 = []
value2 = []

value1_angle = []
value2_angle = []

def newOdom(msg):
    global laser
    laser = msg.ranges

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
   global w, theta_o, theta_g, do, dg

   v = msg.linear.x
   w = msg.angular.z


rospy.init_node("speed_controller")

sub = rospy.Subscriber("robot1/base_scan", LaserScan, newOdom)
sub1 = rospy.Subscriber("robot1/odom", Odometry, newOdom1)
sub2 = rospy.Subscriber("robot1/cmd_vel", Twist, newOdom2)
pub = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=1)


r = rospy.Rate(30)
goal = Point()
speed = Twist()
goal.x = -11.0
goal.y = 0.44

i = 0
k = 0
while not rospy.is_shutdown():

    if i > 5:
       # v = list(v)
       # w = list(w)
        inc_x = goal.x-x1
        inc_y = goal.y-y1
        angle_to_goal = atan2(inc_y, inc_x)
        theta_g = atan2(sin(angle_to_goal-theta1), cos(angle_to_goal - theta1))
        inc_ox = obstacle_x-x1
        inc_oy = obstacle_y-y1
        angle_to_ob = atan2(inc_oy, inc_ox)
        theta_o = atan2(sin(angle_to_ob-theta1), cos(angle_to_ob - theta1))
        dx_g = goal.x - inc_x
        dy_g = goal.y - inc_y
        dx_o = obstacle_x - inc_ox
        dy_o = obstacle_y - inc_oy
        dg = sqrt(dx_g*dx_g + dy_g*dy_g)
        do = sqrt(dx_o*dx_o + dy_o*dy_o)
        print("v=", v)
        print("w=", w)
        print("goal.x=")
        #laser = list(laser)
        print('steps:', i)
        #value1 = list([x, y])
        #value2 = value1 + list(laser)
        #print('value4:',value2)

        """  theta_g Required angle for the robot to move in the direction of goal
        theta_o= Required angle for the robot avoid the collision with obstacle
        do: Euclidean distance of robot with the obstacle
        dg: Euclidean distance of robot with the goal
        """

        value1_angle = list([theta_g]) + list([dg]) + list([theta_o]) + list([do]) + list([v])
        value2_angle = list([theta_g]) + list([dg]) + list([theta_o]) + list([do]) + list([w])
        value1_angle = np.array(value1_angle)
        value1_angle = value1_angle.astype('float32')
        value1 = ((value1_angle - nor_min) / (nor_max - nor_min))
        value1 = np.array(value1)
        value2_angle = np.array(value2_angle)
        value2_angle = value2_angle.astype('float32')
        value2 = ((value2_angle - nor_min) / (nor_max - nor_min))
        value2 = np.array(value2)
        value1 = value1.reshape((1, 1, 5))
        value2 = value2.reshape((1, 1, 5))
        print('value1=',value1)
        print ('value2=', value2)
        temp_results = v_model.predict((value1))
        temp_results1 = w_model.predict((value2))

        print('temp_results=',temp_results)
        print('temp_results1=',temp_results1)
        v_pred = (temp_results * nor_max + (nor_min * (1 - temp_results)))
        w_pred = (temp_results1 * nor_max + (nor_min * (1 - temp_results1)))
      #  v_pred = math.floor(v_pred * 10) / 10
        print('v_pred',v_pred)
    #    w_pred = math.floor(w_pred * 10) / 10
        print ('w_pred',w_pred)
        speed.angular.z = w_pred
        speed.linear.x = v_pred
        pub.publish(speed)
        r.sleep()
    i=i+1