#ifndef AIR_ARM_HW_H
#define AIR_ARM_HW_H

#include <iostream>
#include <vector>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "AdsLib.h"
#include "AdsVariable.h"

namespace air_arm_hardware_interface
{

class AirArmHardware: public hardware_interface::RobotHW
{

protected:
    ros::NodeHandle nh_;

    //interfaces
    hardware_interface::JointStateInterface      joint_state_interface_;
    hardware_interface::PosVelAccJointInterface   position_joint_interface_;

    int num_joints_;
    std::vector<std::string> joint_names_;

    //actual states
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;

    //given setpoints
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> joint_acc_command_;

}; // class

} // namespace

#endif // AIR_ARM_HW_H
