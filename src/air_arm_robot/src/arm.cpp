#include "air_arm_robot/arm.h"
#include "air_arm_robot/arm_ik.h"

namespace air {

Arm::Arm()
{
  std::cout << "ARM constructor()" << std::endl;
  AdsVariable<uint32_t> ads_last_timestamp {route, "GVL.fLastTimeStamp"};
  AdsVariable<bool> ads_halt {route, "GVL.state_HALT"};
  AdsVariable<bool> ads_timeout {route, "GVL.state_TIMEOUT"};
  ads_last_timestamp = ros::Time::now().sec;
  ads_act_timestamp_ = ros::Time::now().sec;
  ads_halt = false;
  ads_timeout = false;
}

bool Arm::setJointNames(std::vector<std::string> names){
  joint_names_ = names;
}

sensor_msgs::JointState Arm::readJoint(int num_joints, std::array<double, 7> &raw_joint_positions ){

  sensor_msgs::JointState joint_state;

  std::array<double, 7> posArray;
  std::array<double, 7> velArray;

  try {

    AdsVariable<std::array<double, 7> > posVar {route, "GVL.fActPose_Arm"};
    AdsVariable<std::array<double, 7> > velVar {route, "GVL.fActVel_Arm"};
    posArray = posVar;
    velArray = velVar;

    raw_joint_positions = posArray;

  } catch (const AdsException& ex) {
      std::cout << "Error: " << ex.errorCode << "\n";
      std::cout << "AdsException message: " << ex.what() << "\n";
      // TODO: Manage commucation interruption error (Broken pipe)

  } catch (const std::runtime_error& ex) {
      std::cout << ex.what() << '\n';
  }
  // The unit of Joint states is degree at Beckhoff side
  // For s1,s1,s3 joints
  for(int i=0; i< 3; i++){
      joint_state.position.push_back(toRadian(posArray[i]));
      joint_state.velocity.push_back(toRadian(velArray[i]));
  }
    // For elbow joint
      joint_state.position.push_back(arm_ik.elbowLinear2Radian_Position(posArray[3]));
      joint_state.velocity.push_back(arm_ik.elbowLinear2Radian_Velocity(velArray[3]));

    // For roll,pitch joints

      std::pair<double, double> actual_rp_euler_pos;
      std::pair<double, double> actual_rp_euler_vel;
      actual_rp_euler_pos = arm_ik.wristLinear2RadianPosition(std::pair<double, double>(posArray[4],posArray[5]));
      actual_rp_euler_vel = arm_ik.wristLinear2RadianVelocity(std::pair<double, double>(velArray[4],velArray[5]));

      joint_state.position.push_back(actual_rp_euler_pos.first);
      joint_state.velocity.push_back(actual_rp_euler_vel.first);
      joint_state.position.push_back(actual_rp_euler_pos.second);
      joint_state.velocity.push_back(actual_rp_euler_vel.second);

    // For yaw joint
      joint_state.position.push_back(toRadian(posArray[6]));
      joint_state.velocity.push_back(toRadian(velArray[6]));

  return joint_state;
}
bool Arm::writeJoint(int num_joints, std::vector<double> act_pose, std::vector<double> pose, std::vector<double> vel, std::vector<double> acc){

  try {
    AdsVariable<std::array<double, 7> > ads_pos_var {route, "GVL.fGoalPos_Arm"};
    AdsVariable<std::array<double, 7> > ads_vel_var {route, "GVL.fGoalVel_Arm"};
    std::array<double, 7> posToWrite;
    std::array<double, 7> velToWrite;

    // For s1,s1,s3 joints
    for(int i=0; i< 3; i++){
        posToWrite[i] = toDegree(pose.at(i));
        velToWrite[i] = toDegree(vel.at(i));
    }
    // For elbow joint
    // To solve URDF Beckhoff mismatch
        double beckhoff_urdf_error = M_PI / 2.0; // 90 degree disparity
        double elbow_desired_pos = pose.at(3);

        elbow_desired_pos += beckhoff_urdf_error;

        posToWrite[3] = arm_ik.elbowRadian2Linear_Position(elbow_desired_pos);
        velToWrite[3] = arm_ik.elbowRadian2Linear_Velocity(elbow_desired_pos,vel.at(3));
    /*
     * For roll,pitch joints
     *
     * Convert roll and pitch positions(radian) to linear distances
     * Calculate delta time for each joint from planned trajectory (using desired and actual joint states (delta_θ = w * t))
     * Calculate linear velocity of each joint in linear space
    */
    std::pair<double, double> desired_rp_angular_pos, desired_rp_angular_vel,
                              desired_rp_linear_pos, desired_rp_linear_vel;
    // convert roll and pitch positions(radian) to linear distance
    desired_rp_angular_pos = std::make_pair(pose.at(4),pose.at(5));
    desired_rp_angular_vel = std::make_pair(vel.at(4),vel.at(5));
    desired_rp_linear_pos = arm_ik.wristEuler2LinearPosition(desired_rp_angular_pos);
    desired_rp_linear_vel = arm_ik.wristEuler2LinearVelocity(desired_rp_angular_pos,
                                                             desired_rp_angular_vel);

    // roll joint
    posToWrite[4] = desired_rp_linear_pos.first; // in mm
    velToWrite[4] = desired_rp_linear_vel.first; // in mm/s
    // pitch joint
    posToWrite[5] = desired_rp_linear_pos.second; // in mm
    velToWrite[5] = desired_rp_linear_vel.second; // in mm/s
    // For yaw joint
    posToWrite[6] = toDegree(pose.at(6));
    velToWrite[6] = toDegree(vel.at(6));

    ads_pos_var = posToWrite;
    ads_vel_var = velToWrite;

    ads_act_timestamp_ = ros::Time::now().sec;

  } catch (const AdsException& ex) {
      std::cout << "Error: " << ex.errorCode << "\n";
      std::cout << "AdsException message: " << ex.what() << "\n";
      // TODO: Manage commucation interruption error (Broken pipe)


  } catch (const std::runtime_error& ex) {
      std::cout << ex.what() << '\n';
  }
//  std::cout << "Arm::writeJoint" << std::endl;
  return true;
}

double Arm::toRadian(double degree){

  return (degree / 180.0) * M_PI;
}

double Arm::toDegree(double radian){

  return (radian / M_PI) * 180.0;
}

} // namespace

