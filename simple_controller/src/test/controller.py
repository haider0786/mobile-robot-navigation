#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from math import sin
from math import cos
#from pandas import read_csv
from pandas import DataFrame
from pandas import concat
x = 0.0
y = 0.0
theta = 0.0
x_final = -5
y_final = 6.0
x1 = 0.0
y1 = 0.0

def newOdom(msg):
    global x
    global y
    global theta
    global x_final
    global y_final
    

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


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

sub = rospy.Subscriber("robot1/odom", Odometry, newOdom)
sub = rospy.Subscriber("robot2/odom", Odometry, newOdom1)
pub = rospy.Publisher("robot2/cmd_vel", Twist, queue_size=1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()

goal.x = x_final 
goal.y = y_final
init_x = 0.0
init_y = 0.0
init_x1 = 0.0
init_y1 = 0.0
vx = 0.0
vy  = 0.0
vx1 = 0.0
vy1 = 0.0
counter = 0
while not rospy.is_shutdown():
    inc_x = goal.x-x
    inc_y = goal.y-y
    angle_to_goal = atan2(inc_y, inc_x)
    init_vx = vx
    init_vy = vy
    init_vx1 = vx1
    init_vy1 = vy1

    vx = x - init_x
    vy = y - init_y
    vx1 = x1 - init_x1
    vy1 = y1 - init_y1
    ax = vx - init_vx
    ay = vy - init_vy
    ax1 = vx1 - init_vx1
    ay1 = vy1 - init_vy1

    init_x = x
    init_y = y
    init_x1 = x1
    init_y1 = y1

    #print(x,y)
    #print(x1, y1)
    z = [x, y, vx, vy, ax, ay, x1, y1, vx1, vy1, ax1, ay1]
    #z = z.reshape((1, 1, 4))
    print(z)
    if counter > 1:
        print( 'yes i am in prediction=', counter)
    counter = counter + 1

    # if (atan2(sin(angle_to_goal-theta), cos(angle_to_goal-theta)))>0.1:
    #     speed.linear.x = 0.0
    #     speed.angular.z = 0.3
    # elif(atan2(sin(angle_to_goal-theta), cos(angle_to_goal-theta)))<(-0.1):
    #     speed.linear.x = 0.0
    #     speed.angular.z = -0.3
    # else:
    #     speed.linear.x = 0.5
    #     speed.angular.z = 0.0
    # pub.publish(speed)
    # if(x>=x_final and y>=y_final):
    #     speed.linear.x = 0.0
    #     speed.angular.z = 0.0
    #     pub.publish(speed)
   	#rospy.shutdown()
    r.sleep()


