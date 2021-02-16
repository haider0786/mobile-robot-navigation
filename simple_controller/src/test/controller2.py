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





'''controller...............................................................................................................'''

x = 0.0
y = 0.0
theta = 0.0
x_final = 6.0
y_final = 6.0
e = 0.0
kw = 1.0
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

r = rospy.Rate(4)

goal = Point()

goal.x = 0.0
goal.y = 0.0

while not rospy.is_shutdown():
    if goal.x != x_final and goal.y != y_final:
        goal.x = goal.x+1
        goal.y = goal.y+1
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





