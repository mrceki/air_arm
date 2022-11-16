#include <air_arm_hardware_interface/air_arm_hw_interface.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "air_arm_hardware_interface_node");
  ros::NodeHandle nh;


  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(2);
  spinner.start();

  air_arm_hardware_interface::AirArmHardwareInterface arm_hw_interface(nh);

  ros::waitForShutdown();

//  ros::spin();

  return 0;
}
