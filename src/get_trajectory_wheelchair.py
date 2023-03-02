import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import pickle as pkl
import numpy as np
import sys
import matplotlib.pyplot as plt

odometry_topic_name = '/wcias_controller/odom'
start_time_topic_name = '/start_record_wheelchair'
stop_time_topic_name = '/stop_record_wheelchair'

start_time = 0.0
stop_time = 1e100

trj = []

filename = 'wheelchair_trajectory.pkl'

def cbk_odom(data):
    global trj
    pose = data.pose.pose
    twist = data.twist.twist
    time = data.header.stamp
    sec_time = time.secs + time.nsecs * (10 ** (-9))
    trj.append([sec_time, pose.position.x, pose.position.y, pose.position.z, twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z])
    #print(sec_time)

def cbk_start_time(data):
    global start_time
    start_time = data.data
    print(start_time)

def cbk_stop_time(data):
    global stop_time
    stop_time = data.data
    print(stop_time)

def display_and_save():
    global trj, start_time, stop_time
    trj_cut = []
    for point in trj:
        if point[0] >= start_time and point[0] <= stop_time:
            trj_cut.append(point)
    trj_np = np.array(trj_cut)
    pkl.dump(trj_np, open(filename, 'wb'))

    plt.figure('2d trajectory')
    plt.scatter(trj_np[:, 1], trj_np[:, 2])
    plt.show()


if __name__ == '__main__':
    rospy.init_node('get_trajectory_wheelchair_node')

    if len(sys.argv) > 1:
        filename = sys.argv[1]

    # odometry listener
    rospy.Subscriber(odometry_topic_name, Odometry, cbk_odom)

    # start time listener
    rospy.Subscriber(start_time_topic_name, Float32, cbk_start_time)

    # stop time listener
    rospy.Subscriber(stop_time_topic_name, Float32, cbk_stop_time)

    rospy.on_shutdown(display_and_save)

    rospy.spin()

