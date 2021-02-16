#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from math import atan2
from math import sqrt
from math import sin
from math import cos
import sys
import numpy as np
import pandas as pd
from numpy import inf
laser1 = []
laser2 = []

def newlaser1(msg):
    global laser1
    laser1 = msg.ranges

def newlaser2(msg):
    global laser2
    laser2 = msg.ranges

def stop1(laser1, val1):
    for x1 in list(laser1):
        if x1 > val1:
            print("x1=", x1)
            return 1

def stop2(laser2, val2):
    for x2 in list(laser2):
        if x2 > val2:
            print("x1=", x2)
            return 1



rospy.init_node("dy_controller")
speed1 = Twist()
speed2 = Twist()
r = rospy.Rate(2)
speed1.linear.x = 0.5
speed1.angular.z = 0.0

speed2.linear.x = 0.5
speed2.angular.z = 0.0

sub1 = rospy.Subscriber('/Obstacle1/base_scan', LaserScan, newlaser1)
pub1 = rospy.Publisher('/Obstacle1/cmd_vel', Twist, queue_size=1)

sub2 = rospy.Subscriber('/Obstacle2/base_scan', LaserScan, newlaser2)
pub2 = rospy.Publisher('/Obstacle2/cmd_vel', Twist, queue_size=1)

i= 0
while not rospy.is_shutdown():
    laser1 = np.array(laser1[545:515])
    laser1[laser1 == inf] = 30
    print(' laser1= ', laser1)

    laser2 = np.array(laser2[545:515])
    laser2[laser2 == inf] = 30
    print(' laser2= ', laser2)

    if stop1(laser1, 4):
        pub1.publish(speed1)
        print('i am inside in 1')
    else:
        speed1.linear.x = 0
        speed1.angular.z = 0
        pub1.publish(speed1)
        print('i am outside of 1')
        


    if stop2(laser2, 4):
        pub2.publish(speed2)
        print('i am inside in 2')
    else:
        speed2.linear.x = 0
        speed2.angular.z = 0
        pub2.publish(speed2)
        print('i am outside of 2')
        
          
    speed1.linear.x = 0.8
    speed1.angular.z = 0.0

    speed2.linear.x = 0.8
    speed2.angular.z = 0.0
    i = i+1


    r.sleep()







