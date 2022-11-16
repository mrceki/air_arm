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
    # Close Connection
    plc.close()
    print ("Connection Closed")

########### POSITION / VELOCITY LIMITS ###################
# AXIS 1 =  -45 <> 45   /
# AXIS 2 =  -90 <> 1    /
# AXIS 3 =  -90 <> 90   /


def main(path=1):

    if(len(sys.argv)>1):
        path = int(sys.argv[1])
    add_route()
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

        state_RUNNING = plc.read_by_name("GVL.state_RUNNING", pyads.PLCTYPE_BOOL)
        print("state_RUNNING(Should be FALSE): ", state_RUNNING)

        # plc.write_by_name("GVL.bArmEnable", [False, False, False], pyads.PLCTYPE_BOOL * 3)
        # plc.write_by_name("GVL.bArmPositive", [False, False, False], pyads.PLCTYPE_BOOL * 3)
        # plc.write_by_name("GVL.bArmNegative", [False, False, False], pyads.PLCTYPE_BOOL * 3)

        bArmEnable = plc.read_by_name("GVL.bArmEnable", pyads.PLCTYPE_BOOL * 3)
        bArmPositive = plc.read_by_name("GVL.bArmPositive", pyads.PLCTYPE_BOOL * 3)
        bArmNegative = plc.read_by_name("GVL.bArmNegative", pyads.PLCTYPE_BOOL * 3)
        print("bArmEnable: ", bArmEnable)
        print("bArmPositive: ", bArmPositive)
        print("bArmNegative: ", bArmNegative)
        # time.sleep(0.2)
        plc.write_by_name("GVL.bArmEnable", [True, True, True], pyads.PLCTYPE_BOOL * 3)
        plc.write_by_name("GVL.bArmPositive", [True, True, True], pyads.PLCTYPE_BOOL * 3)
        plc.write_by_name("GVL.bArmNegative", [True, True, True], pyads.PLCTYPE_BOOL * 3)

        if(path == 1):
            print("path1")
            plc.write_by_name("GVL.fGoalPos_Arm", [40.0, 0.0 , 0.0], pyads.PLCTYPE_LREAL * 3)

        elif(path == 2):
            print("path2")
            plc.write_by_name("GVL.fGoalPos_Arm", [-40.0, -85.0, -85.0], pyads.PLCTYPE_LREAL * 3)

        plc.write_by_name("GVL.state_RUNNING", True, pyads.PLCTYPE_BOOL)
        state_RUNNING = plc.read_by_name("GVL.state_RUNNING", pyads.PLCTYPE_BOOL)
        print("state_RUNNING(Should be TRUE): ",state_RUNNING)

        # time.sleep(15)
        #
        # plc.write_by_name("GVL.state_RUNNING", False, pyads.PLCTYPE_BOOL)
        # state_RUNNING = plc.read_by_name("GVL.state_RUNNING", pyads.PLCTYPE_BOOL)
        # print("state_RUNNING(Should be FALSE): ",state_RUNNING)

        # plc.write_by_name("GVL.bArmEnable", [False, False, False], pyads.PLCTYPE_BOOL * 3)
        # plc.write_by_name("GVL.bArmPositive", [False, False, False], pyads.PLCTYPE_BOOL * 3)
        # plc.write_by_name("GVL.bArmNegative", [False, False, False], pyads.PLCTYPE_BOOL * 3)
        #
        # bArmEnable = plc.read_by_name("GVL.bArmEnable", pyads.PLCTYPE_BOOL * 3)
        # bArmPositive = plc.read_by_name("GVL.bArmPositive", pyads.PLCTYPE_BOOL * 3)
        # bArmNegative = plc.read_by_name("GVL.bArmNegative", pyads.PLCTYPE_BOOL * 3)
        # print("bArmEnable: ", bArmEnable)
        # print("bArmPositive: ", bArmPositive)
        # print("bArmNegative: ", bArmNegative)
    else:
        print ("PLC NOT CONNECTED!")
        plc.close()
        exit(99)
    plc.close()

if __name__ == '__main__':
    main()