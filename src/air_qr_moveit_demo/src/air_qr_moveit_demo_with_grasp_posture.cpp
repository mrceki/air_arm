/* Author: brkygkcn */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "air_qr_moveit_demo/air_qr_moveit_demo.h"

#include <tf/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <visualization_msgs/Marker.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <string>

#include <control_msgs/FollowJointTrajectoryAction.h>

# define OPEN_QBHAND 0.0
# define CLOSE_QBHAND 0.9

namespace rvt = rviz_visual_tools;

/*
 *  adjust goal orientation to end effector orientation
 */
void adjustGoalPose(geometry_msgs::Pose &pose){

  tf2::Quaternion q_orig, q_rot, q_new;
  // Get the original orientation of 'commanded_pose'
  tf2::convert(pose.orientation, q_orig);

  q_rot.setRPY(1.5708,0, 0);
  q_new = q_rot*q_orig;
  q_new.normalize();

  q_rot.setRPY(0, 0, 1.5708);
  q_new = q_rot*q_new;
  q_new.normalize();

  tf2::convert(q_new, pose.orientation);
}

void convertTFtoPose(tf::StampedTransform &qr_transform,
                        tf::StampedTransform &base_transform,
                        geometry_msgs::Pose &pose){

  pose.position.x = qr_transform.getOrigin().x();
  pose.position.y = qr_transform.getOrigin().y();
  pose.position.z = qr_transform.getOrigin().z();
  pose.orientation.x = base_transform.getRotation().x();
  pose.orientation.y = base_transform.getRotation().y();
  pose.orientation.z = base_transform.getRotation().z();
  pose.orientation.w = base_transform.getRotation().w();
}

class AirDemo{
private:
  const std::string PLANNING_GROUP_ = "right_arm";
  const std::string  VISUALIZATION_FRAME_ = "world";
  const std::string QBHAND_ACTION = "/qbhand1/control/qbhand1_synergy_trajectory_controller/follow_joint_trajectory";
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
  Eigen::Affine3d text_pose_;
  const robot_state::JointModelGroup* joint_model_group_;
  std::vector<moveit_msgs::CollisionObject> collision_objects_;


  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> qb_hand_client_;
public:
  AirDemo();
  ~AirDemo();
  void textVisualization(std::string text);
  void planVisualization(moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
                                geometry_msgs::Pose &goal_pose);
  void pickDemo();
  bool pick(std::string &goal_tf_name);
  bool setGoal(geometry_msgs::Pose &goal_pose, std::string goal_tf);
  bool setPlan(moveit::planning_interface::MoveGroupInterface::Plan &plan,
               geometry_msgs::Pose &goal_pose);
  bool move(moveit_msgs::Grasp &grasp);

  bool actionQBHand(double goal_pose, double duration);
  bool setPickPlan(geometry_msgs::Pose &goal_pose, moveit_msgs::Grasp &grasp);
  void addCollisionObjects(geometry_msgs::Pose &object_pose);
  void removeCollisionObjects();
  void openGripper(trajectory_msgs::JointTrajectory& posture, double duration);
  void closeGripper(trajectory_msgs::JointTrajectory& posture, double duration);

};

AirDemo::AirDemo():move_group_(PLANNING_GROUP_),
                   visual_tools_(VISUALIZATION_FRAME_),
                   qb_hand_client_(QBHAND_ACTION, true){
    move_group_.setPlanningTime(45.0);
    joint_model_group_ = move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP_);
    collision_objects_.resize(0);
    planning_scene_interface_.applyCollisionObjects(collision_objects_);
    text_pose_ = Eigen::Affine3d::Identity();
    text_pose_.translation().z() = 1.75;

    // BEGIN_DEMO
    textVisualization("AIR Demo");
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to Start the demo");

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    ROS_INFO_NAMED("AIR DEMO", "Reference frame: %s", move_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("AIR DEMO", "End effector link: %s", move_group_.getEndEffectorLink().c_str());

    textVisualization("Start the demo");

}

AirDemo::~AirDemo(){

}

bool AirDemo::move(moveit_msgs::Grasp &grasp){
  std::cout << " -- Moving to Goal --"<< std::endl;
  /*return (move_group_.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);*/
  return (move_group_.pick("object", grasp)== moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

bool AirDemo::pick(std::string &goal_tf_name){
/*
  geometry_msgs::Pose pick_pose;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to setGoal...");
  if(setGoal(pick_pose, goal_tf_name)){
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to setPlan...");
    if(setPlan(plan, pick_pose)){
      visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to move...");
      if(move()){
        return actionQBHand(CLOSE_QBHAND, 2.0);
      }
    }
  }
*/
  geometry_msgs::Pose pick_pose;
  moveit_msgs::Grasp grasp;
  visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to setGoal...");
  if(setGoal(pick_pose, goal_tf_name)){
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to setPickPlan...");
    if(setPickPlan(pick_pose, grasp)){
      visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to move...");
      if(move(grasp)){
        //return actionQBHand(CLOSE_QBHAND, 2.0);
        return true;
      }
    }
  }
  removeCollisionObjects();
  return false;
}

bool AirDemo::setPickPlan(geometry_msgs::Pose &goal_pose, moveit_msgs::Grasp &grasp){

  grasp.grasp_pose.header.frame_id = "base_link";
  grasp.grasp_pose.pose = goal_pose;
  // Setting pre-grasp approach
  grasp.pre_grasp_approach.direction.header.frame_id = "base_link";
  /* Direction is set as negative z axis */
  grasp.pre_grasp_approach.direction.vector.z = -1.0;
  grasp.pre_grasp_approach.min_distance = 0.095;
  grasp.pre_grasp_approach.desired_distance = 0.115;
  // Setting post-grasp retreat
  grasp.post_grasp_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as positive z axis */
  grasp.post_grasp_retreat.direction.vector.z = 1.0;
  grasp.post_grasp_retreat.min_distance = 0.05;
  grasp.post_grasp_retreat.desired_distance = 0.15;
  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasp.pre_grasp_posture, 2.0);
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closeGripper(grasp.grasp_posture, 2.0);
  return true;
}

void AirDemo::addCollisionObjects(geometry_msgs::Pose &object_pose)
{
  collision_objects_.resize(1);

  //Define the object that we will be manipulating
  collision_objects_[0].id = "object";
  collision_objects_[0].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects_[0].primitives.resize(1);
  collision_objects_[0].primitives[0].type = collision_objects_[0].primitives[0].BOX;
  collision_objects_[0].primitives[0].dimensions.resize(3);
  collision_objects_[0].primitives[0].dimensions[0] = 0.1;
  collision_objects_[0].primitives[0].dimensions[1] = 0.15;
  collision_objects_[0].primitives[0].dimensions[2] = 0.1;

  /* Define the pose of the object. */
  collision_objects_[0].primitive_poses.resize(1);
  collision_objects_[0].primitive_poses[0] = object_pose;
  collision_objects_[0].operation = collision_objects_[0].ADD;

  planning_scene_interface_.applyCollisionObjects(collision_objects_);
}

void AirDemo::removeCollisionObjects(){
  std::vector<std::string> object_ids;

  for (moveit_msgs::CollisionObject obj : collision_objects_){
    object_ids.push_back(obj.id);
  }
  planning_scene_interface_.removeCollisionObjects(object_ids);
}

void AirDemo::openGripper(trajectory_msgs::JointTrajectory& posture, double duration)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(1);
  posture.joint_names[0] = "yaw";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = OPEN_QBHAND;
  posture.points[0].time_from_start = ros::Duration(duration);
}

void AirDemo::closeGripper(trajectory_msgs::JointTrajectory& posture, double duration)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(1);
  posture.joint_names[0] = "yaw";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = CLOSE_QBHAND;
  posture.points[0].time_from_start = ros::Duration(duration);
}

bool AirDemo::setPlan(moveit::planning_interface::MoveGroupInterface::Plan &plan,
                      geometry_msgs::Pose &goal_pose){
  bool result;
  move_group_.setStartStateToCurrentState();
  move_group_.setPoseTarget(goal_pose);
  result = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("AIR DEMO", "Visualizing plan 1 (qr goal) %s", result ? "" : "FAILED");
  if(result){
    planVisualization(plan,goal_pose);
  }
  return result;
}

// GO TO PICK
void AirDemo::pickDemo(){

  std::string goal_tf_name("/fiducial_100");
  bool run_demo = true;
  bool result;
  char user_input;

  do {
    //actionQBHand(OPEN_QBHAND, 2.0);
    std::cout << "pickDemo Started...";
    result = pick(goal_tf_name);
    ROS_INFO_NAMED("AIR DEMO", "Picking is %s", result ? "COMPLETED" : "FAILED");

    std::cout << "You can press 'q' to quit..." << std::endl;
    user_input = std::cin.get();
    if(user_input=='Q'  || user_input=='q'){
      std::cout << "Terminating..." << std::endl;
      run_demo = false;
    }

  }while(run_demo);

  return;
}

bool AirDemo::setGoal(geometry_msgs::Pose &goal_pose, std::string goal_tf_name){

  tf::TransformListener listener;
  tf::StampedTransform goal_tf, baselink_tf;

  try {

    listener.waitForTransform("/base_link", goal_tf_name, ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("/base_link", goal_tf_name, ros::Time(0), goal_tf);

    listener.waitForTransform("/world", "/base_link", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("/world", "/base_link", ros::Time(0), baselink_tf);

  } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      return false;
  }

  convertTFtoPose(goal_tf, baselink_tf, goal_pose);
  adjustGoalPose(goal_pose);


  visual_tools_.deleteAllMarkers();
  visual_tools_.publishAxisLabeled(goal_pose, "pick_pose");
  visual_tools_.publishText(text_pose_, "Pick Pose", rvt::WHITE, rvt::XLARGE);
  visual_tools_.trigger();

  ROS_INFO_NAMED("AIR DEMO", "Detected T %.2f %.2f %.2f ",
                              goal_pose.position.x,
                              goal_pose.position.y,
                              goal_pose.position.z);
  visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to addCollisionObjects...");
  addCollisionObjects(goal_pose);
  return true;
}


void AirDemo::textVisualization(std::string text)
{
  visual_tools_.deleteAllMarkers();
  visual_tools_.publishText(text_pose_, text, rvt::WHITE, rvt::XLARGE);
  visual_tools_.trigger();
}

void AirDemo::planVisualization(moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
                                geometry_msgs::Pose &goal_pose){
  ROS_INFO_NAMED("AIR DEMO", "Visualizing plan as trajectory line");
  visual_tools_.deleteAllMarkers();
  visual_tools_.publishAxisLabeled(goal_pose, "goal_pose");
  visual_tools_.publishText(text_pose_, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_);
  visual_tools_.trigger();
}

bool AirDemo::actionQBHand(double goal_pose, double duration){

  if(qb_hand_client_.waitForServer(ros::Duration(5.0)))
  {
    bool finished_before_timeout;
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.resize(1);
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(1);
    goal.trajectory.joint_names[0] = "qbhand1_synergy_joint";
    goal.trajectory.points[0].positions[0] = goal_pose;
    goal.trajectory.points[0].time_from_start = ros::Duration(duration);
    goal.goal_time_tolerance = ros::Duration(0.0);
    qb_hand_client_.sendGoal(goal);
    finished_before_timeout = qb_hand_client_.waitForResult(ros::Duration(5.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = qb_hand_client_.getState();
      ROS_INFO_STREAM("QBHand Action finished: " << state.getText());
    }
    else
    {
      ROS_INFO_STREAM("QBHand Action did not finish before the time out.");
      return false;
    }
  }
  else
  {
    ROS_INFO_STREAM("qb_hand_action_server NOT AVAILABLE!!!");
    return false;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "air_arm_pick_place");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  AirDemo demo;

  demo.pickDemo();

  //ros::waitForShutdown();

  return 0;
}

/*
 * moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  // joint group demo First Goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the joint group demo");

  std::vector<double> joint_group_positions;

  std::vector<double> path1 = {-40, -80, -80, 0.1, -10, -10, -100};

  std::cout << "---- joint group demo First Goal ----" << std::endl;
  for(auto it = path1.begin(); it!=path1.end(); it++){
    joint_group_positions.push_back(brkygkcn::toRadian(*it)); // radians
  }
  for(auto p : joint_group_positions){
    std::cout << "pose: " << brkygkcn::toDegree(p) << std::endl;
  }
  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "To First Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  std::cout << " -- Moving to First Goal"<< std::endl;
  move_group.move();

  current_state = move_group.getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  std::cout << "-- Current Robot State --" << std::endl;
  for(auto it = joint_group_positions.begin(); it!=joint_group_positions.end(); it++){
    std::cout << brkygkcn::toDegree(*it) << std::endl;
  }
  std::cout << " -- "<< std::endl;

  // Second Goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Go to second Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Since we set the start state we have to clear it before planning other paths

//  std::vector<double> path2 = { 40,   0,  80,  70,  10,  10,  100};
  std::vector<double> path2 = { 40,   0,  80,  0.1, -10, -10, -100};

  joint_group_positions.clear();

  std::cout << "---- Second Goal ----" << std::endl;
  for(auto it = path2.begin(); it!=path2.end(); it++){
    joint_group_positions.push_back(brkygkcn::toRadian(*it)); // radians
  }

  for(auto p : joint_group_positions){
    std::cout << "pose: " << brkygkcn::toDegree(p) << std::endl;
  }

  move_group.setJointValueTarget(joint_group_positions);
  success = false;
  int attempt = 0;
  do {

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Attempt %d", attempt);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  attempt++;

  }while(!success && attempt < 3);

  if(success) {

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "To Second Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


    std::cout << " -- Moving to Second Goal"<< std::endl;
    move_group.move();

    current_state = move_group.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    std::cout << "-- Current Robot State --" << std::endl;
    for(auto it = joint_group_positions.begin(); it!=joint_group_positions.end(); it++){
      std::cout << brkygkcn::toDegree(*it) << std::endl;
    }
    std::cout << " -- "<< std::endl;

  }

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to terminate the demo");

  // END_TUTORIAL

/*
  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose.
  robot_state::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group.setPoseTarget(target_pose1);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose3);

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Wait for MoveGroup to recieve and process the collision object message
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // Now when we plan a trajectory it will avoid the obstacle
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.w = 1.0;
  another_pose.position.x = 0.4;
  another_pose.position.y = -0.4;
  another_pose.position.z = 0.9;
  move_group.setPoseTarget(another_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // Now, let's attach the collision object to the robot.
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group.attachObject(collision_object.id);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to recieve and process the attached collision object message *-/
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
                      "robot");

  // Now, let's detach the collision object from the robot.
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group.detachObject(collision_object.id);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to recieve and process the attached collision object message *-/
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
                      "robot");

  // Now, let's remove the collision object from the world.
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to recieve and process the attached collision object message *-/
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");

  // END_TUTORIAL

  ros::shutdown();*/

