#include <air_arm_hardware_interface/air_arm_hw_interface.h>
#include <string>

namespace air_arm_hardware_interface
{


AirArmHardwareInterface::AirArmHardwareInterface(ros::NodeHandle& nh): nh_(nh){

  init();
  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

  nh_.param("/arm/hardware_interface/loop_hz", loop_hz_, 0.1);
  ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
  non_realtime_loop_ = nh_.createTimer(update_freq, &AirArmHardwareInterface::update, this);

  ROS_DEBUG_NAMED("hardware_interface arm/arm_position_controller", "Loaded generic_hardware_interface.");
}

AirArmHardwareInterface::~AirArmHardwareInterface(){

}

void AirArmHardwareInterface::init(){

  nh_.getParam("/arm/hardware_interface/joints", joint_names_);
  if (joint_names_.size() == 0)
  {
    ROS_FATAL_STREAM_NAMED("init","No joints found on parameter server for controller. Did you load the proper yaml file?");
  }
  num_joints_ = joint_names_.size();

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_acc_command_.resize(num_joints_);

  //Register handles
  for(int i=0; i<num_joints_; i++){

    // Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);


    // Create position joint interface
    hardware_interface::PosVelAccJointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]
                                                                                 , &joint_velocity_command_[i]
                                                                                 , &joint_acc_command_[i]);

    position_joint_interface_.registerHandle(jointPositionHandle);
  }

  //Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
}

void AirArmHardwareInterface::update(const ros::TimerEvent& e) {

  elapsed_time_ = ros::Duration(e.current_real - e.last_real);

  read();
  controller_manager_->update(e.current_real, elapsed_time_);
  write(elapsed_time_);

  _logInfo = "\n";
  _logInfo += "Joint Position Error:\n";
  for (int i=0; i<num_joints_; i++)
  {
    std::ostringstream jointPositionStr;
    jointPositionStr << joint_position_command_[i] - joint_position_[i];
    _logInfo += "  " + joint_names_[i] + ": " + jointPositionStr.str() + "\n";
  }

//  ROS_INFO_STREAM_THROTTLE(5.0,_logInfo);
}

void AirArmHardwareInterface::read(){
    _logInfo = "\n";
    _logInfo += "Joint State Read:\n";
    std::array<double, 7> raw_joint_positions;
    sensor_msgs::JointState act_joint_states = arm.readJoint(num_joints_, raw_joint_positions);

    for(int i=0; i<num_joints_; i++){
        joint_position_[i] = act_joint_states.position[i];
        joint_velocity_[i] = act_joint_states.velocity[i];

        std::ostringstream jointPositionStr, jointRawPositionStr, jointVelocityStr;
        jointPositionStr << joint_position_[i];
        jointRawPositionStr << raw_joint_positions[i];
        jointVelocityStr << joint_velocity_[i];
        _logInfo += "  " + joint_names_[i] + ": " + jointPositionStr.str()+ " - " + jointRawPositionStr.str() + " - " +jointVelocityStr.str() + "\n";
    }
    ROS_INFO_STREAM_THROTTLE(2.0,_logInfo);
}

void AirArmHardwareInterface::write(const ros::Duration& elapsed_time){

    arm.writeJoint(num_joints_, joint_position_, joint_position_command_, joint_velocity_command_, joint_acc_command_);

    _logInfo = "\n";
    _logInfo += "Joint Position Write:\n";

    for(int i=0; i<num_joints_ ;i++){

        std::ostringstream jointPositionStr, jointVelocityStr;
        jointPositionStr << joint_position_[i];
        jointVelocityStr << joint_position_command_[i];
        _logInfo += "  " + joint_names_[i] + ": " + jointPositionStr.str()+ " - " + jointVelocityStr.str() + "\n";
    }

    ROS_INFO_STREAM_THROTTLE(2.0,_logInfo);
}

} // namespace
