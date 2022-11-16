#!/usr/bin/env python
'''ros_ads ROS Node'''
# license removed for brevity
import pyads
import rospy
import numpy as np
import time

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
# AXIS 4 =    0 <> 80   /
# AXIS 5 =  -15 <> 15   / 7.5 mm/s
# AXIS 6 =  -15 <> 15   / 7.5 mm/s
# AXIS 7 = -200 <> 200  / 80  mm/s

if __name__ == '__main__':

    path = 3
    add_route()
    plc = pyads.Connection('192.168.66.29.1.1', pyads.PORT_TC3PLC1) #BeckHoff AmsNetId
    plc.open()

    t = 0
    timeout = 20
    while (not plc.is_open and t < timeout):
        print("Waiting for connection...")
        time.sleep(0.5)
        t+=1

    if(plc.is_open):
        print ("PLC CONNECTED!")

        bRunBlocks = plc.read_by_name("GVL.bRunBlocks", pyads.PLCTYPE_BOOL)
        print("bRunBlocks(Should be FALSE): ", bRunBlocks)

        plc.write_by_name("GVL.bArmEnable", [False, False, False, False, False, False, False], pyads.PLCTYPE_BOOL * 7)
        plc.write_by_name("GVL.bArmPositive", [False, False, False, False, False, False, False], pyads.PLCTYPE_BOOL * 7)
        plc.write_by_name("GVL.bArmNegative", [False, False, False, False, False, False, False], pyads.PLCTYPE_BOOL * 7)

        bArmEnable = plc.read_by_name("GVL.bArmEnable", pyads.PLCTYPE_BOOL * 7)
        bArmPositive = plc.read_by_name("GVL.bArmPositive", pyads.PLCTYPE_BOOL * 7)
        bArmNegative = plc.read_by_name("GVL.bArmNegative", pyads.PLCTYPE_BOOL * 7)
        print("bArmEnable: ", bArmEnable)
        print("bArmPositive: ", bArmPositive)
        print("bArmNegative: ", bArmNegative)
        # time.sleep(0.2)
        plc.write_by_name("GVL.bArmEnable", [True, True, True, True, True, True, True], pyads.PLCTYPE_BOOL * 7)
        plc.write_by_name("GVL.bArmPositive", [True, True, True, True, True, True, True], pyads.PLCTYPE_BOOL * 7)
        plc.write_by_name("GVL.bArmNegative", [True, True, True, True, True, True, True], pyads.PLCTYPE_BOOL * 7)

        if(path == 1):
            plc.write_by_name("GVL.fPos_Arm", [40.0, 0.0 , 85.0, 0.1 , 13.0, 13.0, 150.0], pyads.PLCTYPE_LREAL * 7)
            plc.write_by_name("GVL.fVel_Arm", [20.0, 20.0, 20.0, 10.0, 5.0, 5.0, 30.0], pyads.PLCTYPE_LREAL * 7)
            plc.write_by_name("GVL.fAcc_Arm", [5.0 , 5.0 , 5.0 , 4.0 , 3.0 , 3.0 , 15.0], pyads.PLCTYPE_LREAL * 7)
        elif(path == 2):
            plc.write_by_name("GVL.fPos_Arm", [-40.0, -85.0, -85.0, 75.0, -13.0, -13.0, -150.0], pyads.PLCTYPE_LREAL * 7)
            plc.write_by_name("GVL.fVel_Arm", [20.0 ,  20.0,  20.0, 10.0,  5.0,   5.0 ,  30.0], pyads.PLCTYPE_LREAL * 7)
            plc.write_by_name("GVL.fAcc_Arm", [5.0  ,  5.0 ,  5.0 , 4.0 ,  3.0 ,  3.0 ,  15.0], pyads.PLCTYPE_LREAL * 7)
        elif(path == 3):
            plc.write_by_name("GVL.fPos_Arm", [0.0, -45.0 , 0.0, 40.0, 0.0, 0.0, 0.0], pyads.PLCTYPE_LREAL * 7)
            plc.write_by_name("GVL.fVel_Arm", [20.0, 20.0, 20.0, 10.0, 5.0, 5.0, 30.0], pyads.PLCTYPE_LREAL * 7)
            plc.write_by_name("GVL.fAcc_Arm", [5.0 , 5.0 , 5.0 , 4.0 , 3.0 , 3.0 , 15.0], pyads.PLCTYPE_LREAL * 7)

        plc.write_by_name("GVL.bRunBlocks", True, pyads.PLCTYPE_BOOL)
        bRunBlocks = plc.read_by_name("GVL.bRunBlocks", pyads.PLCTYPE_BOOL)
        print("bRunBlocks: ",bRunBlocks)
        # plc.write_by_name("GVL.bArmEnable", [False, False, False, False, False, False, False], pyads.PLCTYPE_BOOL * 7)
        # plc.write_by_name("GVL.bArmPositive", [False, False, False, False, False, False, False], pyads.PLCTYPE_BOOL * 7)
        # plc.write_by_name("GVL.bArmNegative", [False, False, False, False, False, False, False], pyads.PLCTYPE_BOOL * 7)

        bArmEnable = plc.read_by_name("GVL.bArmEnable", pyads.PLCTYPE_BOOL * 7)
        bArmPositive = plc.read_by_name("GVL.bArmPositive", pyads.PLCTYPE_BOOL * 7)
        bArmNegative = plc.read_by_name("GVL.bArmNegative", pyads.PLCTYPE_BOOL * 7)
        print("bArmEnable: ", bArmEnable)
        print("bArmPositive: ", bArmPositive)
        print("bArmNegative: ", bArmNegative)
    else:
        print ("PLC NOT CONNECTED!")
        plc.close()
        exit(99)
    plc.close()
