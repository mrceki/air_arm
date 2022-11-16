#!/usr/bin/env python

# subscribe encoder data
# subscribe imu data
# output to csv file


import rospy
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from sensor_msgs.msg import Imu
import numpy as np
from tf.transformations import euler_from_quaternion

def toDegree(radian):
    return (radian/np.pi)*180.0

def callback(data):
    acc_x = data.linear_acceleration.x
    acc_y = data.linear_acceleration.y
    acc_z = data.linear_acceleration.z

    roll = np.arctan(acc_y / np.sqrt(acc_x**2 + acc_z**2))
    pitch = np.arctan(acc_x / np.sqrt(acc_y**2 + acc_z**2))

    print("roll: ", toDegree(roll))
    print("pitch: ", toDegree(pitch))
    print('\n')

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('imu_listener', anonymous=True)

    rospy.Subscriber("/imu/data", Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

