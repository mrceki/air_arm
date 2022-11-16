#!/usr/bin/env python
'''ros_ads ROS Node'''
# license removed for brevity
import pyads
import rospy
import numpy as np
import time
import sys
import threading

from geometry_msgs.msg import Vector3Stamped

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
    ros_thread.join()

########### POSITION / VELOCITY LIMITS ###################
# AXIS 1 =  -45 <> 45   /
# AXIS 2 =  -90 <> 1    /
# AXIS 3 =  -90 <> 90   /

def wait_for_DONE(plc):
   timeout = 120
   t = 0
   running = True
   while (running and t < timeout):
       time.sleep(0.5)
       running = plc.read_by_name("GVL.bRunWRIST", pyads.PLCTYPE_BOOL)
       t+=1
   if(running == False):
       print("Done...")
   else:
       print("Timeout error!!!")

def go_to_home_pose(plc):
    plc.write_by_name("GVL.fPos_Wrist", [0.0, 0.0 , 0.0], pyads.PLCTYPE_LREAL * 3)
    plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)
    run(plc)
    wait_for_DONE(plc)

def run(plc):
    state_RUNNING = plc.read_by_name("GVL.bRunWRIST", pyads.PLCTYPE_BOOL)
    print("bRunWRIST: ", state_RUNNING)
    if(not state_RUNNING):
        plc.write_by_name("GVL.bRunWRIST", True, pyads.PLCTYPE_BOOL)

def move(plc):
    run(plc)
    wait_for_DONE(plc)
    go_to_home_pose(plc)



def toROS():
    global run_ros_thread
    plc = pyads.Connection('192.168.66.29.1.1', pyads.PORT_TC3PLC1) # BeckHoff AmsNetId
    plc.open()

    pub = rospy.Publisher('wrist_encoder_rpy', Vector3Stamped, queue_size=10)

    while run_ros_thread and not rospy.is_shutdown():
       try:
           pitch = round(plc.read_by_name("GVL.pitch_pos", pyads.PLCTYPE_LREAL), 4)
           roll = round(plc.read_by_name("GVL.roll_pos", pyads.PLCTYPE_LREAL), 4)

#           print("pitch: ", pitch)
#           print("roll: " , roll)

           rpy_msg = Vector3Stamped()
           rpy_msg.vector.x = pitch
           rpy_msg.vector.y = roll
           rpy_msg.vector.z = 0.0
           rpy_msg.header.stamp = rospy.Time.now()

           pub.publish(rpy_msg)

           time.sleep(0.1)
       except KeyboardInterrupt:
           print ("KeyboardInterrupt caught")
           plc.close()
           raise KeyboardInterrupt


run_ros_thread = True

if __name__ == '__main__':

#    if(len(sys.argv)>1):
#        path = int(sys.argv[1])
    # add_route()
    plc = pyads.Connection('192.168.66.29.1.1', pyads.PORT_TC3PLC1) # BeckHoff AmsNetId
    plc.open()

    rospy.init_node('wrist_encoder_rpy_node', anonymous=True, disable_signals=True)

    ros_thread = threading.Thread(target= toROS )
    ros_thread.deamon = True
    ros_thread.start()


    t = 0
    timeout = 30
    while (t < timeout):
        print("Waiting for connection...")
        time.sleep(0.5)
        t+=1

    if(plc.is_open):
        try:

            print ("PLC CONNECTED!")
            print("path0 - Home")
            go_to_home_pose(plc)

            print("path1")
            plc.write_by_name("GVL.fPos_Wrist", [0.0, 15.0 , 0.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            move(plc)

            print("path2")
            plc.write_by_name("GVL.fPos_Wrist", [0.0, 0.0 , 15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            move(plc)

            print("path3")
            plc.write_by_name("GVL.fPos_Wrist", [0.0, -15.0 , 0.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            move(plc)

            print("path4")
            plc.write_by_name("GVL.fPos_Wrist", [0.0, 0.0 , -15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            move(plc)

            print("path5")
            plc.write_by_name("GVL.fPos_Wrist", [0.0, 15.0 , 15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            move(plc)

            print("path6")

            plc.write_by_name("GVL.fPos_Wrist", [0.0, -15.0 , -15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            move(plc)

            print("path7")

            plc.write_by_name("GVL.fPos_Wrist", [0.0, 15.0 , -15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            move(plc)

            print("path8")

            plc.write_by_name("GVL.fPos_Wrist", [0.0, -15.0 , 15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            move(plc)

            print("path9")

            plc.write_by_name("GVL.fPos_Wrist", [0.0, -15.0 , -15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            run(plc)
            wait_for_DONE(plc)

            plc.write_by_name("GVL.fPos_Wrist", [0.0, 15.0 , 15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            move(plc)

            print("path10")

            plc.write_by_name("GVL.fPos_Wrist", [0.0, -15.0 , 15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            run(plc)
            wait_for_DONE(plc)

            plc.write_by_name("GVL.fPos_Wrist", [0.0, 15.0 , -15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            move(plc)

            print("path11")

            plc.write_by_name("GVL.fPos_Wrist", [0.0, -15.0 , -15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            run(plc)
            wait_for_DONE(plc)

            plc.write_by_name("GVL.fPos_Wrist", [0.0, -15.0 , 15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            run(plc)
            wait_for_DONE(plc)

            plc.write_by_name("GVL.fPos_Wrist", [0.0, 15.0 , 15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            run(plc)
            wait_for_DONE(plc)

            plc.write_by_name("GVL.fPos_Wrist", [0.0, 15.0 , -15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            run(plc)
            wait_for_DONE(plc)

            plc.write_by_name("GVL.fPos_Wrist", [0.0, -15.0 , -15.0], pyads.PLCTYPE_LREAL * 3)
            plc.write_by_name("GVL.fVel_Wrist", [0.0, 3.0 , 3.0], pyads.PLCTYPE_LREAL * 3)

            move(plc)

            state_RUNNING = plc.read_by_name("GVL.bRunWRIST", pyads.PLCTYPE_BOOL)
            print("state_RUNNING(Should be FALSE): ",state_RUNNING)

        except (KeyboardInterrupt, rospy.ROSInterruptException) :
            print("Ctrl-C handling and send kill to threads-1")
            global run_ros_thread
            run_ros_thread = False
            print("1")
            rospy.signal_shutdown("reason")
            print("2")

    else:
        print ("PLC NOT CONNECTED!")
        plc.close()
        exit(99)
    plc.close()
