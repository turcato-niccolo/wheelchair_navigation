import rospy
from nav_msgs.msg import Odometry
import numpy as np
import sys
from geometry_msgs.msg import Twist
import math
import PotentialFieldPlanning.Navigation as Navigation
num_cells = 10
num_sectors = 11
max_radius = 4.0  # meters
max_angle = np.pi/2  # +/-


odometry_topic_name = '/wcias_controller/odom'
cmd_topic_name = '/wcias_controller/cmd_vel'

x_goal = 4.0
y_goal = 2.0

v_long = 0.5
gain_w = 1.0

epsilon = 0.1

pub_vel = None

def cbk_odom(data):
    global num_cells, num_sectors
    pose = data.pose.pose

    x_curr = pose.position.x
    y_curr = pose.position.y

    nav = Navigation.PolarPotentialFieldNav(num_cells, num_sectors)
    dist = np.sqrt((x_goal-x_curr) ** 2 + (y_goal-y_curr) ** 2)
    nav.set_goal(dist, math.atan2(y_goal-y_curr, x_goal-x_curr))

    theta = nav.get_heading()

    msg = Twist()
    if dist > epsilon:
        msg.linear.x = v_long
        msg.angular.z = gain_w * theta
    else:
        msg.linear.x = 0.0
        msg.angular.z = 0.0

    pub_vel.publish(msg)

    if dist <= epsilon:
        exit(0)

if __name__ == '__main__':
    rospy.init_node('potentials_wheelchair_nav')

    if len(sys.argv) > 2:
        x_goal = float(sys.argv[1])
        y_goal = float(sys.argv[2])
    pub_vel = rospy.Publisher('cmd_vel', Twist)

    # odometry listener
    rospy.Subscriber(odometry_topic_name, Odometry, cbk_odom)

    rospy.spin()
