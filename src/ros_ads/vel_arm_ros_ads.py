#!/usr/bin/env python
'''ros_ads ROS Node'''
# license removed for brevity
import pyads
import rospy
import numpy as np
import time
import sys

from geometry_msgs.msg import Twist

# Connect to BeckHoff

def add_route():
    pyads.open_port()
    pyads.set_local_address('192.168.66.101.1.1')

    AMS = '192.168.66.101.1.1'
    PLC = '192.168.66.29'
    USER = 'Administrator'
    PW = '1'
    HOSTNAME = '192.168.66.99'
    PLC_AMS_ID = '192.168.66.29.1.1'
    pyads.add_route_to_plc(AMS, HOSTNAME, PLC, USER, PW)

def __del__(self):
    print ("___Process Termination___")

    # Close Connection
    plc.close()
    print ("PLC Connection Closed")

########### POSITION / VELOCITY LIMITS ###################
# AXIS 1 =  -45 <> 45   /
# AXIS 2 =  -90 <> 1    /
# AXIS 3 =  -90 <> 90   /


def main(path=1):

    if(len(sys.argv)>1):
        path = int(sys.argv[1])
#    add_route()
    plc = pyads.Connection('192.168.66.29.1.1', pyads.PORT_TC3PLC1) # BeckHoff AmsNetId
    plc.open()

    t = 0
    timeout = 20
    while (not plc.is_open and t < timeout):
        print("Waiting for connection...")
        time.sleep(0.5)
        t+=1

    if(plc.is_open):
        print ("PLC CONNECTED!")
        # ROS ACTIVE

        fActPose_Arm = plc.read_by_name("GVL.fActPose_Arm", pyads.PLCTYPE_LREAL * 7)
        print("\nfActPose_Arm: ", fActPose_Arm)

        time.sleep(0.5)

#        plc.write_by_name("GVL.fGoalVel_Arm", [5.0, 5.0, 5.0, 3.0, 3.0, 3.0, 15.0], pyads.PLCTYPE_LREAL * 7)
#        plc.write_by_name("GVL.fGoalVel_Arm", [-.0, -5.0, -5.0, -3.0, -3.0, -3.0, -15.0], pyads.PLCTYPE_LREAL * 7)
        plc.write_by_name("GVL.fGoalVel_Arm", [0.0, -0.0, 0.0, -0.0, 3.0, -3.0, 0.0], pyads.PLCTYPE_LREAL * 7)
        time.sleep(8.0) # Run for 10 seconds

        plc.write_by_name("GVL.fGoalVel_Arm", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], pyads.PLCTYPE_LREAL * 7)

        fActPose_Arm = plc.read_by_name("GVL.fActPose_Arm", pyads.PLCTYPE_LREAL * 7)
        print("\nfActPose_Arm: ", fActPose_Arm)

    else:
        print ("PLC NOT CONNECTED!")

if __name__ == '__main__':
    main()
