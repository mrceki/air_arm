#!/usr/bin/env python

# subscribe encoder data
# subscribe imu data
# output to csv file


import rospy
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import TwistStamped
#from sensor_msgs.msg import Imu
import message_filters
import numpy as np
import atexit
import csv

f = open('ik_wrist.csv', 'w')
writer = csv.writer(f)

def callback(enc_data, qr_data):

#    print("Enc : ", str(enc_data.header.stamp))
#    print("QR : ", str(qr_data.header.stamp))
#    print("FARK: ", str(enc_data.header.stamp - qr_data.header.stamp ))

    row = [enc_data.vector.x, enc_data.vector.y, qr_data.twist.angular.x, qr_data.twist.angular.y, qr_data.twist.angular.z]
    writer.writerow(row)



@atexit.register
def goodbye():
    f.close()
    print("Goodbye!!!")


if __name__ == '__main__':
    rospy.init_node("ik_wrist_csv_writer_node", anonymous=True)

    encoder_sub = message_filters.Subscriber("/wrist_encoder_rpy", Vector3Stamped)
    qr_sub = message_filters.Subscriber("/qr_pose", TwistStamped)

    ts = message_filters.ApproximateTimeSynchronizer([encoder_sub, qr_sub], queue_size=1000, slop=0.1)
    ts.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
