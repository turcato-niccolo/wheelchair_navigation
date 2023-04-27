import copy

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, TwistStamped
import pickle as pkl
import numpy as np
import sys
import matplotlib.pyplot as plt
import message_filters
import std_msgs.msg

costmap_topic = '/move_base/local_costmap/costmap'
th_costmap_topic = '/move_base/local_costmap/costmap_threshold'

pub_th = None


def cbk_costmap(costmap):
    global pub_th
    print(type(costmap.data))
    th_costmap = list(costmap.data)

    cnt = 0

    for i in range(len(th_costmap)):
        d = th_costmap[i]
        if d >= 100:
            th_costmap[i] = 100
            cnt += 1
        else:
            th_costmap[i] = 0


    print(cnt)

    costmap.data = tuple(th_costmap)

    pub_th.publish(costmap)


if __name__ == '__main__':
    rospy.init_node('wheelchair_costmap_th')

    pub_th = rospy.Publisher(costmap_topic, OccupancyGrid, queue_size=1)
    rospy.Subscriber(costmap_topic, OccupancyGrid, cbk_costmap)

    rospy.spin()


