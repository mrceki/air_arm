#ifndef ARM_H
#define ARM_H

#include "air_arm_robot/arm_ik.h"

#include "AdsLib.h"
#include "AdsVariable.h"

#include <iostream>
#include <vector>
#include <iomanip>
#include <array>
#include <sensor_msgs/JointState.h>


# define M_PI           3.14159265358979323846  /* pi */

namespace air {

class Arm
{
private:
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;

  AmsNetId remoteNetId { 192, 168, 6, 29, 1, 1 };
  const std::string remoteIpV4 = "192.168.6.29";

//  AdsSetLocalAddress({192, 168, 0, 1, 1, 1});

  AdsDevice route{remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};


public:
  Arm();
  Arm_IK arm_ik;
  AdsVariable<uint32_t> ads_act_timestamp_ {route, "GVL.fActTimeStamp"};
  bool setJointNames(std::vector<std::string> names);

  sensor_msgs::JointState readJoint(int num_joints_, std::array<double, 7> &raw_joint_positions);
  bool writeJoint(int num_joints_,
                  std::vector<double> act_pose,
                  std::vector<double> pose,
                  std::vector<double> vel,
                  std::vector<double> acc);

  double calcDeltaT(double actual_pose, double desired_pose, double desired_velocity);

  double toRadian(double degree);
  double toDegree(double radian);
};

} // namespace



#endif // ARM_H
