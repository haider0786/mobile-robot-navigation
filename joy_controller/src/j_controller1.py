#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Joy


joy_w = 0.0
joy_f = 0.0

def newjoy(msg):
    global joy_w
    global joy_f
    joy_w = msg.axes[0]
    joy_f = msg.axes[1]


rospy.init_node("joy_controller")

sub0 = rospy.Subscriber('/joy', Joy, newjoy)

pub0 = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=1)

speed = Twist()




while not rospy.is_shutdown():
    print('joy_w and joy_f', joy_w, joy_f)
    speed.linear.x = joy_f
    speed.angular.z = joy_w
    if speed.linear.x >= 1:
        speed.linear.x = 1
    pub0.publish(speed)
    print('speed', speed)





