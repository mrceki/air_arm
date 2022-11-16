#ifndef AIR_ARM_HW_INTERFACE_H
#define AIR_ARM_HW_INTERFACE_H

#include <air_arm_hardware_interface/air_arm_hw.h>
#include <air_arm_robot/arm.h>

namespace air_arm_hardware_interface
{

class AirArmHardwareInterface: public air_arm_hardware_interface::AirArmHardware
{
  public:
    AirArmHardwareInterface(ros::NodeHandle& nh);
    ~AirArmHardwareInterface();
    void init();
    void update(const ros::TimerEvent& e);
    void read();
    void write(const ros::Duration& elapsed_time);

  protected:
    air::Arm arm;
    ros::NodeHandle nh_;
    ros::Timer non_realtime_loop_;
    ros::Duration control_period_;
    ros::Duration elapsed_time_;
    hardware_interface::PosVelAccJointInterface positionJointInterface;
    double loop_hz_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    double p_error_, v_error_, e_error_;
    std::string _logInfo;

};// class

} // namespace
#endif // AIR_ARM_HW_INTERFACE_H
