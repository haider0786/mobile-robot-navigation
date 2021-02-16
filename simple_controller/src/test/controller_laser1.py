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
nor_max = 50.0
nor_min =-50.0
laser = []
x1 = 0.0
y1 = 0.0
kw = 0.5
kv = 0.1
dx = 0.0
dy = 0.0
e = 0.0

x_model = load_model('x_model_static_laser.h5')
y_model = load_model('y_model_static_laser.h5')


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
r = rospy.Rate(1)
speed = Twist()
goal = Point()
goal.x = 0.0
goal.y = 0.0
while not rospy.is_shutdown():

    z = list([x1,y1])
    z_laser = list(laser) + z
    z_laser = np.array(z_laser)
    z_laser = z_laser.astype('float32')
    x_value_nor = [((z_laser - nor_min) / (nor_max - nor_min))]
    print('laser=',z_laser.shape)
    print('x,y=',x1, y1)
    print('counter=', counter)
    print('x_value_nor=',x_value_nor)
    x_value_nor = np.array(x_value_nor)

    z1 = list([y1, x1])
    z1_laser = list(laser) + z1
    z1_laser = np.array(z1_laser)
    z1_laser = z1_laser.astype('float32')
    y_value_nor = [((z1_laser - nor_min) / (nor_max - nor_min))]
    print('laser=', z1_laser.shape)
    print('y_value_nor=', y_value_nor)
    y_value_nor = np.array(y_value_nor)
    length = y_value_nor.shape
    print('length=',length[1])
    if counter >5:
        x_value_nor = x_value_nor.reshape((1, 1, 1022))
        y_value_nor = y_value_nor.reshape((1, 1, 1022))


        temp_results = x_model.predict((x_value_nor))
        temp_results1 = y_model.predict((y_value_nor))
        x_pred = (temp_results * nor_max + (nor_min * (1 - temp_results)))
        y_pred = (temp_results1 * nor_max + (nor_min * (1 - temp_results1)))
        print('temp_results= and x_pred',temp_results,x_pred)
        print('temp_results1= and y_pred', temp_results1, y_pred)

        goal.x = x_pred
        goal.y = y_pred
        inc_x = goal.x - x1
        inc_y = goal.y - y1
        angle_to_goal = atan2(inc_y, inc_x)
        dx = goal.x - inc_x
        dy = goal.y - inc_y
        e = atan2(sin(angle_to_goal - theta1), cos(angle_to_goal - theta1))
        if abs(e) > 0.1:
            speed.angular.z = (kw * e)
            speed.linear.x = 0.0
        # print(x1, y1, theta1, e, goal.x, goal.y)
        else:
            speed.linear.x = kv * sqrt(dx * dx + dy * dy)
            # speed.linear.x = 0.5
            speed.angular.z = 0.0

        pub.publish(speed)


    r.sleep()
    counter = counter +1