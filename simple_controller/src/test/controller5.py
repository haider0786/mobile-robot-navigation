#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
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
x1 = 0.0
y1 = 0.0
theta1 = 0.0
x_final = -5
y_final = 6.0
x2 = 0.0
y2 = 0.0
kw = 0.5
kv = 0.1
dx = 0.0
dy = 0.0
e = 0.0
x_value = 0.0
y_value = 0.0

x_model = load_model('x_model.h5')
y_model = load_model('y_model.h5')

# load dataset...............................................................
dataset = read_csv('new_path_27.csv', header=0)
values = dataset.values
#print(values)
print(values.shape)
# integer encode direction
encoder = LabelEncoder()
values[:, 2] = encoder.fit_transform(values[:, 2])
# ensure all data is float
values = values.astype('float32')
# normalize features
scaler = MinMaxScaler(feature_range=(0, 1))
#scaled = scaler.fit_transform(values)
#........................................................................................


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

    rot_q2 = msg.pose.pose.orientation
    (roll2, pitch2, theta2) = euler_from_quaternion([rot_q2.x, rot_q2.y, rot_q2.z, rot_q2.w])


rospy.init_node("speed_controller")

sub = rospy.Subscriber("robot1/odom", Odometry, newOdom1)
sub = rospy.Subscriber("robot2/odom", Odometry, newOdom2)
pub = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=1)

speed = Twist()

r = rospy.Rate(2000)

goal = Point()

goal.x = x_final
goal.y = y_final
init_x1 = 0.0
init_y1 = 0.0
init_x2 = 0.0
init_y2 = 0.0
vx1 = 0.0
vy1 = 0.0
vx2 = 0.0
vy2 = 0.0
counter = 0
while not rospy.is_shutdown():

    init_vx1 = vx1
    init_vy1 = vy1
    init_vx2 = vx2
    init_vy2 = vy2

    vx1 = x1 - init_x1
    vy1 = y1 - init_y1
    vx2 = x2 - init_x2
    vy2 = y2 - init_y2
    ax1 = vx1 - init_vx1
    ay1 = vy1 - init_vy1
    init_y2 = y2

    ax2 = vx2 - init_vx2
    ay2 = vy2 - init_vy2

    init_x1 = x1
    init_y1 = y1
    init_x2 = x2
    # print(x,y)
    # print(x1, y1)
    z = [x1, y1, vx1, vy1, ax1, ay1, x2, y2, vx2, vy2, ax2, ay2]
    values[0] = z
    scaled = scaler.fit_transform(values)
    z = scaled[0]
    z = z.reshape((1, 1, 12))

    print(z)
    if counter > 1:
        temp_results = x_model.predict((z))
        scaled[0, 0] = temp_results
        temp_results = y_model.predict((z))
        scaled[1, 1] = temp_results
        scaled1 = scaler.inverse_transform(scaled)
        x_value = scaled1[0, 0]
        y_value = scaled1[1, 1]
        print('temp_results=', temp_results)
        print('yes i am in prediction=', counter)
        print('x_value = ', x_value)
        print('y_value=', y_value)

        goal.x = x_value
        goal.y = y_value
        inc_x = goal.x - x1
        inc_y = goal.y - y1
        angle_to_goal = atan2(inc_y, inc_x)
        dx = goal.x - inc_x
        dy = goal.y - inc_y
        e = atan2(sin(angle_to_goal - theta1), cos(angle_to_goal - theta1))
        if abs(e) > 0.1:
            speed.angular.z = (kw * e)
            speed.linear.x = 0.0
            print(x1, y1, theta1, e, goal.x, goal.y)
        else:
            speed.linear.x = kv * sqrt(dx * dx + dy * dy)
        # speed.linear.x = 0.5
            speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep()
    counter = counter + 1


