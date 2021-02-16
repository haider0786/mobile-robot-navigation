#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from math import sin
from math import cos
from math import sqrt
import numpy as np

x = 0.0
y = 0.0
theta = 0.0
x_final = 2.0
y_final = 8.0
e = 0.0
kw = 1.0
kv = 1.0


def newOdom(msg):
    global x
    global y
    global theta
    global x_final
    global y_final
    global e
    global kw
    global kv

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


rospy.init_node("speed_controller")

sub = rospy.Subscriber("robot2/odom", Odometry, newOdom)
pub = rospy.Publisher("robot2/cmd_vel", Twist, queue_size=1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()

goal.x = x_final
goal.y = y_final

while not rospy.is_shutdown():
    inc_x = goal.x - x
    inc_y = goal.y - y
    angle_to_goal = atan2(inc_y, inc_y)
    e = atan2(sin(angle_to_goal - theta), cos(angle_to_goal - theta))
    if abs(e) > 0.05:
        speed.angular.z = (kw * e)
        speed.linear.x = 0.0
        print(x, y, theta, e)
    else:
        #speed.linear.x = kv*sqrt((goal.x-inc_x)**2 + (goal.y-inc_y)**2)
        speed.linear.x = 0.5
        speed.angular.z = 0.0

    pub.publish(speed)
    r.sleep()




