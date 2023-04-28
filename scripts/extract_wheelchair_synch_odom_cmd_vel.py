import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped
import pickle as pkl
import numpy as np
import sys
import matplotlib.pyplot as plt
import message_filters
import std_msgs.msg

odometry_topic_name = '/odom'
cmd_vel_topic_name = '/cmd_vel_stamped'

filename = 'wheelchair_odom_cmd.pkl'
record_velocities = []
record_inputs = []

def callback(data_odom, data_cmd):
    global record_velocities, record_inputs

    twist = data_odom.twist.twist
    time_odom = data_odom.header.stamp
    sec_time_odom = time_odom.secs + time_odom.nsecs * (10 ** (-9))
    record_velocities.append([sec_time_odom, twist.linear.x, twist.angular.z])

    time_stamp = data_cmd.header.stamp
    sec_time_cmd = time_stamp.secs + time_stamp.nsecs * (10 ** (-9))

    print(f'{sec_time_cmd} - {sec_time_odom}')

    record_inputs.append([sec_time_cmd, data_cmd.twist.linear.x, data_cmd.twist.angular.z])


def display_and_save():
    global record_velocities, record_inputs

    velocities = np.array(record_velocities)
    inputs = np.array(record_inputs)

    print(velocities)

    pkl.dump([velocities, inputs], open(filename, 'wb'))

    plt.figure('Linear velocity')
    plt.plot(velocities[:, 0]-velocities[0, 0], velocities[:, 1], label='odom')
    plt.plot(inputs[:, 0] - inputs[0, 0], inputs[:, 1], label='input')
    #plt.stairs(inputs[:-1, 1], inputs[:, 0]-inputs[0, 0], label='input')
    plt.grid()
    plt.legend()

    plt.figure('Angular velocity')
    plt.plot(velocities[:, 0]-velocities[0, 0], velocities[:, 2], label='odom')
    plt.plot(inputs[:, 0] - inputs[0, 0], inputs[:, 2], label='input')
    #plt.stairs(inputs[:-1, 2], inputs[:, 0]-inputs[0, 0], label='input')
    plt.grid()
    plt.legend()

    plt.show()


if __name__ == '__main__':
    rospy.init_node('get_wheelchair_odom_node')

    if len(sys.argv) > 1:
        filename = sys.argv[1]

    cmd_sub = message_filters.Subscriber(cmd_vel_topic_name, TwistStamped)
    odom_sub = message_filters.Subscriber(odometry_topic_name, Odometry)
    ts = message_filters.TimeSynchronizer([odom_sub, cmd_sub], 100)
    ts.registerCallback(callback)

    rospy.on_shutdown(display_and_save)

    rospy.spin()
