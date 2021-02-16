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


kw = 0.5
kv = 0.1
dx = 0.0
dy = 0.0
e = 0.0
x_value = 0.0
y_value = 0.0
max_value = 50
min_value = -50
x_model = load_model('x_model_static.h5')
y_model = load_model('y_model_static.h5')

# load dataset...............................................................

# normalize features
#scaler = MinMaxScaler(feature_range=(0, 1))
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




rospy.init_node("speed_controller")

sub = rospy.Subscriber("robot1/odom", Odometry, newOdom1)

pub = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=1)

speed = Twist()

r = rospy.Rate(5)

goal = Point()

goal.x = x_final
goal.y = y_final
init_x1 = 0.0
init_y1 = 0.0
vx1 = 0.0
vy1 = 0.0
counter = 0
while not rospy.is_shutdown():

    init_vx1 = vx1
    init_vy1 = vy1

    vx1 = x1 - init_x1
    vy1 = y1 - init_y1

    ax1 = vx1 - init_vx1
    ay1 = vy1 - init_vy1
    init_x1 = x1
    init_y1 = y1
    euqd = sqrt(pow(-2-x1, 2)+pow(2-y1,2))+1
    # print(x,y)
    # print(x1, y1)

    #...............................
    z = [x1, y1, vx1, vy1, ax1, ay1, euqd]
    z = np.array(z)
    z = z.astype('float32')
    print ('z=', z)
    z_nor = [((z - min_value) / (max_value - min_value))]
    print('z_nor=', z_nor)
    z_nor = np.array(z_nor)

    z1 = [y1, x1, vy1, vx1, ay1, ax1, euqd]
    z1 = np.array(z1)
    z1 = z1.astype('float32')
    print ('z1=', z1)
    z1_nor = [((z1 - min_value) / (max_value - min_value))]
    print('z1_nor=', z1_nor)
    z1_nor = np.array(z1_nor)
    z_nor = z_nor.reshape((1, 1, 7))
    z1_nor = z1_nor.reshape((1, 1, 7))

    print('z=',z)
    print ('z1=',z1)
    if counter > 1:
        temp_results = x_model.predict((z_nor))

        temp_results1 = y_model.predict((z1_nor))

        x_value = (temp_results * max_value + (min_value * (1 - temp_results)))
        y_value = (temp_results1 * max_value + (min_value * (1 - temp_results1)))

        #...................................

        #print('scaled=',scaled.shape)
        #print('scaled1=', scaled1.shape)
        #print('scaled2=', scaled2.shape)
        #print('scaled3=', scaled3.shape)
        print('temp_results, x_value', temp_results,x_value)
        print('temp_results1, y_value', temp_results1,y_value)
        #print('yes i am in prediction=', counter)
        #print('x_value = ', x_value)
        #print('y_value=', y_value)

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
           # print(x1, y1, theta1, e, goal.x, goal.y)
        else:
            speed.linear.x = kv * sqrt(dx * dx + dy * dy)
        # speed.linear.x = 0.5
            speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep()
    counter = counter + 1


