#!/usr/bin/env python
'''ros_ads ROS Node'''
# license removed for brevity
import pyads
import rospy
import numpy as np
import time
import sys

from air_arm_ads.msg import Float64ArrayStamped

########### POSITION / VELOCITY LIMITS ###################
# AXIS 1 =  -45 <> 45   /
# AXIS 2 =  -90 <> 1    /
# AXIS 3 =  -90 <> 90   /


# Connect to BeckHoff
def add_route():
    pyads.open_port()
    pyads.set_local_address('192.168.66.101.1.1')

    AMS = '192.168.66.101.1.1'
    PLC = '192.168.6.29'
    USER = 'Administrator'
    PW = '1'
    HOSTNAME = '192.168.66.101'
    PLC_AMS_ID = '192.168.6.29.1.1'
    pyads.add_route_to_plc(AMS, HOSTNAME, PLC, USER, PW)

def __del__(self):
    # Close Connection
    plc.close()
    print ("Connection Closed")



if __name__ == '__main__':
    add_route()
"""
# Beckhoff Setup
    plc = pyads.Connection('192.168.66.29.1.1', pyads.PORT_TC3PLC1) # BeckHoff AmsNetId
    plc.open()

# ROS setup
    pub = rospy.Publisher('ads_ros_encoder', Float64ArrayStamped, queue_size=10)
    rospy.init_node('ads_ros_encoder_node')
    r = rospy.Rate(100) # 100hz

    t = 0
    timeout = 20
    while (not plc.is_open and t < timeout):
        print("Waiting for 10 seconds to connect...")
        time.sleep(0.5)
        t+=1

    if(plc.is_open):
        print ("PLC CONNECTED!")

        while not rospy.is_shutdown():

           encoders = plc.read_by_name("GVL.fActPose_Arm", pyads.PLCTYPE_LREAL * 3)
           msg = Float64ArrayStamped()
           msg.header.stamp = rospy.Time.now()
           msg.data = encoders
           pub.publish(msg)
           r.sleep()

    else:
        print ("PLC NOT CONNECTED!")
        exit(99)
"""
