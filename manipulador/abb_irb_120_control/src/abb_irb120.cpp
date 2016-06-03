/*
 * abb_irb120.cpp
 *
 *  Created on: 20 de abr. de 2016
 *      Author: kaiser
 */

#include <ros/ros.h>
#include "joint_trajectory_controller/joint_trajectory_controller.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <actionlib/client/simple_action_client.h>
#include "map"
#include <iostream>


using namespace std;


int main(int argc, char** argv){

	ros::init(argc, argv, "send_traj");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(10);
	spinner.start();

	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> move_piernas("joint_trajectory_action", true);
	move_piernas.waitForServer();
	ROS_INFO("Connected to server");

	trajectory_msgs::JointTrajectory traj;

	traj.joint_names.push_back("joint_1");
	traj.joint_names.push_back("joint_2");
	traj.joint_names.push_back("joint_3");
	traj.joint_names.push_back("joint_4");
	traj.joint_names.push_back("joint_5");
	traj.joint_names.push_back("joint_6");


	trajectory_msgs::JointTrajectoryPoint posicion1;
		posicion1.positions.push_back(0);
		posicion1.positions.push_back(0.1);
		posicion1.positions.push_back(0.2);
		posicion1.positions.push_back(0.1);
		posicion1.positions.push_back(0.1);
		posicion1.positions.push_back(0.2);

		posicion1.time_from_start.sec=1;

		traj.points.push_back(posicion1);
		traj.header.stamp = ros::Time::now();
		//Create goal_der and send it to the controller
			control_msgs::FollowJointTrajectoryGoal goal_izq;
			goal_izq.trajectory = traj;

			move_piernas.sendGoal(goal_izq);

			bool finished_within_time_izq = move_piernas.waitForResult(ros::Duration(45.0));
			if (!finished_within_time_izq) {
				move_piernas.cancelGoal();
				ROS_INFO("Timed out achieving goal_der A");
			} else {
				actionlib::SimpleClientGoalState state = move_piernas.getState();
				bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
				if (success)
					ROS_INFO("Action finished: %s", state.toString().c_str());
				else {
					ROS_INFO("Action failed: %s", state.toString().c_str());
					ROS_WARN("Addition information: %s", state.text_.c_str());
				}
			}

			ros::Duration(3).sleep();

		return 0;
	}





