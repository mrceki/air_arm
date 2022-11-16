#! /usr/bin/env python
import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal


def qb_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/qbhand1/control/qbhand1_synergy_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    qb_trajectory = JointTrajectory()
    qb_trajectory.joint_names = ["qbhand1_synergy_joint"]
    qb_trajectory.points.append(JointTrajectoryPoint())
    qb_trajectory.points[0].positions = [0.0]
    qb_trajectory.points[0].time_from_start = rospy.Duration(1)
    qb_trajectory.points.append(JointTrajectoryPoint())
    qb_trajectory.points[1].positions = [0.0]
    qb_trajectory.points[1].time_from_start = rospy.Duration(3)
    qb_trajectory.points.append(JointTrajectoryPoint())
    qb_trajectory.points[2].positions = [0.9]
    qb_trajectory.points[2].time_from_start = rospy.Duration(5)
    qb_trajectory.points.append(JointTrajectoryPoint())
    qb_trajectory.points[3].positions = [0.9]
    qb_trajectory.points[3].time_from_start = rospy.Duration(8)
    qb_trajectory.points.append(JointTrajectoryPoint())
    qb_trajectory.points[4].positions = [0.0]
    qb_trajectory.points[4].time_from_start = rospy.Duration(10)
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = qb_trajectory
    goal.goal_time_tolerance = rospy.Duration(0)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    print ("RESULT: ", client.get_result())
    return True

if __name__ == '__main__':

    try:
        rospy.init_node('qb_client_py')
        qb_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
