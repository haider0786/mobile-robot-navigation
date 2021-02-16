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

kv = 1
kw = 1
x1 = 0.0
y1 = 0.0
theta1 = 0.0
x_goal = 0.0
goal1 = 0.0
goal2 = 0.0
dg=5
def new0dom(msg):
    global x1
    global y1
    global theta1
    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta1) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def path1(msg):
    global goal1
    global goal2
    goal1 = msg.x
    goal2 = msg.y


rospy.init_node("controller1")

sub1 = rospy.Subscriber('/Obstacle1/odom', Odometry, new0dom)
sub3 = rospy.Subscriber('/xy1', Point, path1)
pub1 = rospy.Publisher('/Obstacle1/cmd_vel', Twist, queue_size=1)
pub2 = rospy.Publisher('/Ack1', Int16, queue_size=1)

speed = Twist()
i =0
r = rospy.Rate(1)
r1 = rospy.Rate(1000)
while not rospy.is_shutdown():
    if i > 2:
        dg = 5  # any value greater then .01
    while (dg > 0.05) & (i > 1):

        #print('location(x,y),dg,i', x1, y1, dg, i)
        dx = goal1 - x1
        dy = goal2 - y1
        print("goal_x and goal_y", goal1, goal2)
        print("dx and dy", dx, dy)
        angle_to_goal = atan2(dy, dx)
        e = atan2(sin(angle_to_goal - theta1), cos(angle_to_goal - theta1))
        dg = kv * sqrt(dx * dx + dy * dy)
        if abs(e) > 0.1:
            speed.angular.z = (kw * e)
            speed.linear.x = 0.0
        else:
            v = dg
            vmax = 0.8
            v = max(min(v, vmax), -vmax)
            speed.linear.x = v
            speed.angular.z = 0.0
        pub1.publish(speed)
        r1.sleep()
        #if stop(laser, 0.5):
         #   pub1.publish(speed)
        #else:
            #speed.linear.x = 0
            #speed.angular.z = 0
    if i == 1:
        pub2.publish(1)
        print('visited=', i)
    if dg < 0.05:
        pub2.publish(1)  # sending acknowledgment  to path for next subgoal
        print('visited=', i)

    i = i + 1
    r.sleep()
    speed.linear.x = 0
    speed.angular.z = 0
    pub1.publish(speed)

