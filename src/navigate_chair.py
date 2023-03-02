import math

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import PotentialFieldPlanning.Navigation as Navigation

cmd_topic = 'wcias_controller/cmd_vel'
odom_topic = 'wcias_controller/odom'
ground_truth_topic = '/wheelchair_ground_truth'
num_cells = 10
num_sectors = 11
max_radius = 4.0  # meters
max_angle = np.pi/2  # +/-





if __name__ == '__main__':
    while not rospy.is_shutdown():
        print('Input goal (w.r.t. current pose):\n')
        print('X:=', end='')
        x_goal = float(input())
        print('\n')
        print('Y:=', end='')
        y_goal = float(input())

        try:
            chair_pose = rospy.wait_for_message(ground_truth_topic, Odometry, timeout=5.0)
        except rospy.exceptions.ROSException:
            print('timeout')
            continue
        chair_pose = chair_pose.pose.pose
        chair_pos = [chair_pose.position.x, chair_pose.position.y]
        chair_quat = chair_pose.orientation
        orientation_list = [chair_quat.x, chair_quat.y, chair_quat.z, chair_quat.w]
        (_, _, chair_yaw) = euler_from_quaternion(orientation_list)

        X_goal = chair_pos[0] + np.cos(chair_yaw) * x_goal
        Y_goal = chair_pos[1] + np.sin(chair_yaw) * y_goal

        nav = Navigation.PolarPotentialFieldNav(num_cells, num_sectors)
        nav.set_goal(np.sqrt(x_goal**2 + y_goal**2), math.atan2(y_goal, x_goal))


