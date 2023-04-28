#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
import numpy as np
import sys
from geometry_msgs.msg import Twist
import math
from tf.transformations import euler_from_quaternion


# import PotentialFieldPlanning.Navigation as Navigation
num_cells = 40
num_sectors = 61
max_radius = 5.0  # meters
max_angle = np.pi  # +/-

odometry_topic_name = '/wcias_controller/odom'
cmd_topic_name = '/wcias_controller/cmd_vel'

x_goal = 5.0
y_goal = 0.0

v_long = 0.5
gain_w = 0.5

epsilon = 0.25

pub_vel = None
nav = None

list_of_objects = [(3.0, 0.0)]


class PolarNav:
    """
        Polar coordinates navigation
    """

    def update(self, map):
        self.map = map
        return

    def get_map(self):
        return self.map

    def get_heading(self):
        raise NotImplementedError


class PolarPotentialFieldNav(PolarNav):
    def __init__(self, num_cells, num_sec, max_radius, max_angle, goal_weight=-10.0, obstacle_weight=12.5):
        """
        :param num_cells:    Number of cells for each polar sector
        :param num_sec:    Number of sectors, should be odd, the middle sector is the forward direction
        :return:
        :rtype:
        """
        self.num_cells = num_cells
        self.num_sec = num_sec
        self.max_angle = max_angle
        self.max_radius = max_radius
        self.goal_weight = goal_weight
        self.obstacle_weight = obstacle_weight

        self.map = np.zeros((num_cells, num_sec), float)
        self.angles = np.arange(-self.max_angle, self.max_angle, 2 * self.max_angle / self.num_sec)
        self.angles = self.angles[::-1]

        self.sector_angle = 2 * self.max_angle / self.num_sec
        self.cell_len = self.max_radius / self.num_cells

    def get_heading(self):
        """"""
        idx = np.argmin(np.mean(self.map, axis=0))
        return idx, self.angles[idx]
        # return int((self.num_sec-1)/2)

    def get_polar_indices(self, r, theta):
        theta = min(max(-self.max_angle, theta), self.max_angle)
        sector_angle = 2 * self.max_angle / self.num_sec
        cell_len = self.max_radius / self.num_cells
        sector = math.floor((self.max_angle - theta) / sector_angle)
        cell = self.num_cells - 1 - math.floor(r / cell_len)
        return cell, sector

    def reset(self):
        self.map = np.zeros((self.num_cells, self.num_sec), float)

    def set_goal(self, r, theta):
        cell, sector = self.get_polar_indices(r, theta)

        for i in range(self.num_cells):
            for j in range(self.num_sec):
                r_loc, th_loc = self.get_polar_from_idx(i, j)
                d_sq = self.get_sq_cart_dist_from_polar_coord(r_loc, th_loc, r, theta)
                self.map[i, j] += self.goal_weight * np.exp(-(d_sq / np.abs(self.goal_weight)))

    def get_polar_from_idx(self, i, j):
        r_loc = self.max_radius - (i * self.cell_len + self.cell_len / 2)
        th_loc = self.max_angle - (j * self.sector_angle + self.sector_angle / 2)
        return r_loc, th_loc

    def get_sq_cart_dist_from_polar_coord(self, r1, th1, r2, th2):
        return r1 ** 2 + r2 ** 2 - 2 * r1 * r2 * math.cos(th1 - th2)

    def add_obstacle(self, r, theta, radius=1.0):
        cell, sector = self.get_polar_indices(r, theta)

        sector_angle = 2 * self.max_angle / self.num_sec
        cell_len = self.max_radius / self.num_cells

        for i in range(self.num_cells):
            for j in range(self.num_sec):
                r_loc, th_loc = self.get_polar_from_idx(i, j)
                d_sq = self.get_sq_cart_dist_from_polar_coord(r_loc, th_loc, r, theta)
                if d_sq <= radius ** 2:
                    self.map[i, j] = self.obstacle_weight
                else:
                    self.map[i, j] += self.obstacle_weight * np.exp(-(math.sqrt(d_sq)+radius)**2 / np.abs(5*self.obstacle_weight))


def cbk_odom(data):
    global num_cells, num_sectors, pub_img, bridge, nav
    pose = data.pose.pose

    x_curr = pose.position.x
    y_curr = pose.position.y

    dist = np.sqrt((x_goal - x_curr) ** 2 + (y_goal - y_curr) ** 2)
    _, _, yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    ang = math.atan2(y_goal - y_curr, x_goal - x_curr) - yaw
    nav.reset()
    nav.set_goal(dist, ang)

    for p_o in list_of_objects:
        o_d = np.sqrt((p_o[0] - x_curr) ** 2 + (p_o[1] - y_curr) ** 2)
        o_ang = math.atan2(p_o[1] - y_curr, p_o[0] - x_curr) - yaw
        nav.add_obstacle(o_d, o_ang, 0.5)
    idx, theta = nav.get_heading()

    print('({},{}) - ({},{}) - ({}, {}) - {}: {}'.format(x_curr, y_curr, dist, ang, o_d, o_ang, idx, theta))
    #print('d=({},{}) - gamma={}, yaw={}'.format(x_goal-x_curr, y_goal-y_curr, math.atan2(y_goal - y_curr, x_goal - x_curr), yaw))
    #print(y_goal-y_curr)

    msg = Twist()
    if dist > epsilon:
        msg.linear.x = v_long
        msg.angular.z = gain_w * theta
    else:
        msg.linear.x = 0.0
        msg.angular.z = 0.0

    pub_vel.publish(msg)


if __name__ == '__main__':
    rospy.init_node('potentials_wheelchair_nav')

    if len(sys.argv) > 2:
        x_goal = float(sys.argv[1])
        y_goal = float(sys.argv[2])

    pub_vel = rospy.Publisher(cmd_topic_name, Twist, queue_size=1)

    # Nav Object
    nav = PolarPotentialFieldNav(num_cells, num_sectors, max_radius, max_angle)

    # odometry listener
    rospy.Subscriber(odometry_topic_name, Odometry, cbk_odom)

    rospy.spin()
