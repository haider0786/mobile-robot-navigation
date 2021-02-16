#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Point, Twist

ack = 0
back = 1
def reached_callback(msg):
    global ack
    ack = msg.data

sub = rospy.Subscriber('/Ack4', Int16, reached_callback)
x_value = [-2.54, -2.45]
y_value = [-7.95, -1.104]

goal1 = Point()

def talker():
    pub1 = rospy.Publisher('/xy4', Point, queue_size=1)
    rospy.init_node('Trajectory4', anonymous=True)
    rate = rospy.Rate(1)  # 1hz
    global ack

    n = 0
    while not rospy.is_shutdown():
        if back == ack:
            goal1.x = x_value[n]
            goal1.y = y_value[n]
            pub1.publish(goal1)
            print('ack and i= ', ack, n)
            n = n + 1
        ack = 0
        if n > 1:
            n = 0
        rate.sleep()
if __name__ =='__main__':
    talker()

