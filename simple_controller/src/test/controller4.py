#! /usr/bin/env python
import rospkg
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from math import sin
from math import cos
from math import sqrt
from sklearn.model_selection import train_test_split
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import xlrd
import keras.models
from keras.models import load_model

m = 0
'''..........................................................X Axis..........................................'''

df1 = (np.array(pd.read_excel("path_x_41_Norm.xlsx", sheet_name=1).iloc[0:1600, 3:4], dtype=np.float))
#print(df1.shape)
data_x = np.empty((1596, 4, 1))
for i in range(1596):
    m = i

    for j in range(4):

        data_x[i, j, :] = df1[m, :]
        m = m+1
'''...............................................................................................................'''

'''.................................................Y Axis.......................................................'''
i = 0
j = 0
df2 = (np.array(pd.read_excel("path_y_2.xlsx", sheet_name=0).iloc[0:1600, 3:4], dtype=np.float))
print(df2.shape)
data_y = np.empty((1596, 4, 1))
for i in range(1596):
    n = i

    for j in range(4):

        data_y[i, j, :] = df2[n, :]
        n = n+1
'''...............................................................................................................'''




'''' .....................................Load the Trained Model.....................................................'''

model = load_model('model.h5')
y_model = load_model('y_model.h5')
''' ....................................................................................................................'''


#print(data_x[0, 0:4, 0])
i = 0
max_x = -0.4936
min_x = -5.19031
max_y = 6.04746
min_y = 1.381224
y_value = np.empty(1596)
x_value = np.empty(1596)
'''..............................................Converting data from normalization to real value............................................ '''





'''controller...............................................................................................................'''

x = 0.0
y = 0.0
theta = 0.0
x_final = 6.0
y_final = 6.0
e = 0.0
kw = 0.5
kv = 0.1
dx = 0.0
dy = 0.0


def newOdom(msg):
    global x
    global y
    global theta
    global x_final
    global y_final
    global e
    global kw
    global kv
    global dx
    global dy, l

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


rospy.init_node("speed_controller")

sub = rospy.Subscriber("robot2/odom", Odometry, newOdom)
pub = rospy.Publisher("robot2/cmd_vel", Twist, queue_size=1)

speed = Twist()

r = rospy.Rate(15) #Hz

goal = Point()

goal.x = 0.0
goal.y = 0.0

while not rospy.is_shutdown():
    '''..............................................lstm prediction value..........................................'''
    if i<=1595:
        testing = np.array(data_x[i, 0:4, 0])
        testing = testing.reshape((1, 4, 1))
        temp_results = model.predict(testing)

        testing1 = np.array(data_y[i, 0:4, 0])
        testing1 = testing1.reshape((1, 4, 1))
        temp_results1 = y_model.predict(testing1)

        x_value = temp_results * max_x + (1 - temp_results) * min_x

        y_value = temp_results1 * max_y + (1 - temp_results1) * min_y
    else:
        i = 1595
        testing = np.array(data_x[i, 0:4, 0])
        testing = testing.reshape((1, 4, 1))
        temp_results = model.predict(testing)
        testing1 = np.array(data_y[i, 0:4, 0])
        testing1 = testing1.reshape((1, 4, 1))
        temp_results1 = y_model.predict(testing1)

        x_value = temp_results * max_x + (1 - temp_results) * min_x
        y_value = temp_results1 * max_y + (1 - temp_results1) * min_y


    '''...............................................................................................................'''
    i=i+1
    goal.x = x_value
    goal.y = y_value
    inc_x = goal.x - x
    inc_y = goal.y - y
    angle_to_goal = atan2(inc_y, inc_x)
    dx = goal.x-inc_x
    dy = goal.y-inc_y
    e = atan2(sin(angle_to_goal - theta), cos(angle_to_goal - theta))
    if abs(e) > 0.1:
        speed.angular.z = (kw * e)
        speed.linear.x = 0.0
        print(x, y, theta, e, goal.x, goal.y)
    else:
        speed.linear.x = kv*sqrt(dx*dx + dy*dy)
        #speed.linear.x = 0.5
        speed.angular.z = 0.0

    pub.publish(speed)
    r.sleep()





