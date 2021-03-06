/*
 * hermes_robot_interface.h
 *
 *  Created on: 23/09/2013
 *      Author: vr2user
 */

#ifndef HERMES_ROBOT_INTERFACE_H_
#define HERMES_ROBOT_INTERFACE_H_

#include "ros/ros.h"
#include "hermes_robot_interface/HermesInterface.h"
#include "sensor_msgs/JointState.h"
#include <actionlib/server/action_server.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <hermes_robot_interface/MoveArmAction.h>
#include <moveit/move_group_interface/move_group.h>


typedef actionlib::SimpleActionServer<hermes_robot_interface::MoveArmAction> MoveArmActionServerType;
class HermesRobotInterface
{

private:
	typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
	typedef JTAS::GoalHandle GoalHandle;
	JTAS action_server_;
	GoalHandle active_goal_;
	sensor_msgs::JointState jointState;
	ros::Publisher pub_controller_command_;
	trajectory_msgs::JointTrajectory current_traj_;

public:
	HermesRobotInterface(ros::NodeHandle &nh);
	void init();
	void getJointState(sensor_msgs::JointState &jointState);
	void moveArmCB(const hermes_robot_interface::MoveArmGoalConstPtr& goal);

private:
	void executeTrajCB(GoalHandle gh);



protected:
	HermesInterface hermesinterface;
	ros::NodeHandle node_;
	MoveArmActionServerType move_arm_action_server_;






};

#endif /* HERMES_ROBOT_INTERFACE_H_ */
