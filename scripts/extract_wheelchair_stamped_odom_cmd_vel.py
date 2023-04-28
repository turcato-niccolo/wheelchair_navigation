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
pub_cmd_stamp = None

record_velocities = []  # each element is [time, v_meas, w_meas]
record_inputs = []  # each element is [time, v_in, w_in]

def cbk_odom(data_odom):
    global record_velocities
    twist = data_odom.twist.twist

    time_odom = data_odom.header.stamp
    sec_time_odom = time_odom.secs + time_odom.nsecs * (10 ** (-9))

    #print(sec_time_odom)

    record_velocities.append([sec_time_odom, twist.linear.x, twist.angular.z])


def cbk_cmd(data):
    global pub_cmd_stamp, record_inputs
    time_stamp = data.header.stamp
    sec_time_cmd = time_stamp.secs + time_stamp.nsecs * (10 ** (-9))

    record_inputs.append([sec_time_cmd, data.twist.linear.x, data.twist.angular.z])


def display_and_save():
    global record_velocities, record_inputs

    velocities = np.array(record_velocities)
    inputs = np.array(record_inputs)

    print(velocities[0, 0])
    print(inputs[0, 0])

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

    # Subscribe to twist
    rospy.Subscriber(cmd_vel_topic_name, TwistStamped, cbk_cmd)
    rospy.Subscriber(odometry_topic_name, Odometry, cbk_odom)

    rospy.on_shutdown(display_and_save)

    rospy.spin()

