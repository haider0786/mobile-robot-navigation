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
import numpy as np
import pandas as pd
from pandas import read_csv
from keras.models import load_model

laser = []
x_r = 0.0
y_r = 0.0
x_o1 = 0.0
y_o1 = 0.0
x_o2 = 0.0
y_o2 = 0.0
x_o3 = 0.0
y_o3 = 0.0
x_o4 = 0.0
y_o4 = 0.0
x_o5 = 0.0
y_o5 = 0.0
x_o6 = 0.0
y_o6 = 0.0
co = 0
kw = 0.5
kv = 0.1
dx = 0.0
dy = 0.0
dg = 2.0  # initialize to pass the first loop condition
e = 0.0
v = 0.0
w = 0.0

valueob1 = []
valueob2 = []
valueob3 = []
valueob4 = []
valueob5 = []
valueob6 = []


theta_g_max = 4.0
theta_g_min = -4.0
dg_max = 20.0
dg_min = 0.0


z_model = load_model('dy_pos_angular6_pos_o6.h5')
x_model = load_model('dy_linear.h5')
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

def newOdom2(msg):
    global v
    global w, theta_g, dg
    v = msg.linear.x
    w = msg.angular.z

def callback_o1(msg):
    global x_o1
    global y_o1
    global theta_o1
    x_o1 = msg.pose.pose.position.x
    y_o1 = msg.pose.pose.position.y


def callback_o2(msg):
    global x_o2
    global y_o2
    global theta_o2
    x_o2 = msg.pose.pose.position.x
    y_o2 = msg.pose.pose.position.y

def callback_o3(msg):
    global x_o3
    global y_o3
    global theta_o3
    x_o3 = msg.pose.pose.position.x
    y_o3 = msg.pose.pose.position.y

def callback_o4(msg):
    global x_o4
    global y_o4
    global theta_o4
    x_o4 = msg.pose.pose.position.x
    y_o4 = msg.pose.pose.position.y

def callback_o5(msg):
    global x_o5
    global y_o5
    global theta_o5
    x_o5 = msg.pose.pose.position.x
    y_o5 = msg.pose.pose.position.y


def callback_o6(msg):
    global x_o6
    global y_o6
    global theta_o6
    x_o6 = msg.pose.pose.position.x
    y_o6 = msg.pose.pose.position.y


def euclideanaOrientation(x_e, y_e):
    diff_x = x_r - x_e
    diff_y = y_r - y_e
    angle_to_robot = atan2(diff_y, diff_x)
    # theta_g = atan2(sin(angle_to_goal - theta1), cos(angle_to_goal - theta1))
    eucd = sqrt(diff_x * diff_x + diff_y * diff_y)
    #print 'diff_x diff_y x_r y_r', diff_x, diff_y, x_r, y_r
    #print eucd, angle_to_robot, x_e, y_e
    return(angle_to_robot, eucd)

def normalize( theta_n,  dis_n):
    theta_n = (2 * ((theta_n - theta_g_min) / (theta_g_max - theta_g_min)) - 1)
    dis_n = (2 * ((dis_n - dg_min) / (dg_max - dg_min)) - 1)
    return theta_n, dis_n


rospy.init_node("data_collection")

sub = rospy.Subscriber("robot1/base_scan", LaserScan, newOdom)
sub1 = rospy.Subscriber("robot1/odom", Odometry, newOdom1)
sub2 = rospy.Subscriber("robot1/cmd_vel", Twist, newOdom2)
subo1 = rospy.Subscriber("Obstacle1/odom", Odometry, callback_o1)
subo2 = rospy.Subscriber("Obstacle2/odom", Odometry, callback_o2)
subo3 = rospy.Subscriber("Obstacle3/odom", Odometry, callback_o3)
subo4 = rospy.Subscriber("Obstacle4/odom", Odometry, callback_o4)
subo5 = rospy.Subscriber("Obstacle5/odom", Odometry, callback_o5)
subo6 = rospy.Subscriber("Obstacle6/odom", Odometry, callback_o6)
pub = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=1)


r = rospy.Rate(10)
goal = Point()
speed = Twist()

goal_x = [7.88, -8.49 ]  
goal_y = [4.69, -0.202] 

goal.x = 0.0
goal.y = 0.0
j = 0
i = 0
k = 0

while not rospy.is_shutdown():
    inc_x = goal.x - x_r
    inc_y = goal.y - y_r
    theta_g = atan2(inc_y, inc_x)
    # theta_g = atan2(sin(angle_to_goal - theta1), cos(angle_to_goal - theta1))
    dg = sqrt(inc_x * inc_x + inc_y * inc_y)

    [theta_o1, do1] = euclideanaOrientation(x_o1, y_o1)
    [theta_o2, do2] = euclideanaOrientation(x_o2, y_o2)
    [theta_o3, do3] = euclideanaOrientation(x_o3, y_o3)
    [theta_o4, do4] = euclideanaOrientation(x_o4, y_o4)
    [theta_o5, do5] = euclideanaOrientation(x_o5, y_o5)
    [theta_o6, do6] = euclideanaOrientation(x_o6, y_o6)

    [theta_o1_n, do1_n] = normalize(theta_o1, do1)
    [theta_o2_n, do2_n] = normalize(theta_o2, do2)
    [theta_o3_n, do3_n] = normalize(theta_o3, do3)
    [theta_o4_n, do4_n] = normalize(theta_o4, do4)
    [theta_o5_n, do5_n] = normalize(theta_o5, do5)
    [theta_o6_n, do6_n] = normalize(theta_o6, do6)

    [theta_g_n, dg_n] = normalize(theta_g, dg)

  
    
    Input1 = list([theta_o1, do2, theta_o2, do3, theta_o3, do4, theta_o4, theta_o5, do5, theta_o6, do6])
    Input2 = list([theta_g_n, dg_n])
    Input1 = np.array(Input1)
    Input2 = np.array(Input2)
    print 'Input1_shape', Input1.shape
    print 'Input2_shape', Input2.shape
    
    if co > 2:
        Input1 = Input1.reshape((1, 1, len(Input1)))
        Input2 = Input2.reshape((1, len(Input2)))

        print("Input1", Input1)
        print('Input2', Input2)
        # print('path_dist = ', path_dist)
        angular_velocity = z_model.predict([Input1, Input1, Input2, Input2])
        #linear_velocity = x_model.predict([Input1, Input2, Input2])
        #x_pred = (angular_velocity * a_max + (a_min * (1 - angular_velocity)))
        print('angular_velocity', angular_velocity)
      #  print('linear_velocity', linear_velocity)
        # print('Distance_to_goal =', dg)
        speed.angular.z = angular_velocity
        speed.linear.x = 0.8
        pub.publish(speed)
        if dg <= 2.0:
            goal.x = -8.49
            goal.y = -0.203
        

    r.sleep()
    co = co + 1

            
            







