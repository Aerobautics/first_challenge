/*
 * @author: MAVROS, edited by Stewart Nash
 * @file: first_challenge_movement.cpp
 * @brief: First challenge quadcopter control node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include "spacestate.h"
#include "cellconversion.h"

enum ImageState {NONE_FOUND, OBSERVE, TARGET, FORCE_OFF};
bool isTargeting = false;
bool isObserving = false;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
bool isPoseAcquired;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	current_pose = *msg;
	isPoseAcquired = true;
}

int main(int argc, char **argv)
{
	isPoseAcquired = false;
	ros::init(argc, argv, "first_challenge_movement_node");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &state_cb);
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &local_pos_cb);
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	
	geometry_msgs::PoseStamped pose;

	double pose_x, pose_y;
	double desired_x, desired_y;	
	unsigned long elapsedTime = 0;
	
	// The setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20.0);
	
	// Wait for FCU connection
	while(ros::ok() && !current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}
	// PX4 Pro Flight Stack operates in aerospacee NED coordinate frame
	// MAVROS translates to standard ENU frame
	while (!isPoseAcquired) {

	}
	std::cout << "Starting position (" << current_pose.pose.position.x << ", ";
	std::cout << current_pose.pose.position.y << ")" << std::endl;
	pose.pose.position.x = current_pose.pose.position.x; // 0?
	pose.pose.position.y = current_pose.pose.position.y; // 0?
	pose.pose.position.z = current_pose.pose.position.z; // 2?
	pose.pose.orientation.x = current_pose.pose.orientation.x; // 0?
	pose.pose.orientation.y = current_pose.pose.orientation.y; // 0?
	pose.pose.orientation.z = current_pose.pose.orientation.z; // -0.99?
	pose.pose.orientation.w = -current_pose.pose.orientation.w; // -0.04?
	
	pose.pose.position.z = 2;		
	// Send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}
	// Move this to header
	for (int i = 0; i < NUMBER_OF_WALLS; i++) {
		forbiddenPoints[i] = getPoints(i);
	}

	initializeSpace(pose.pose.position.x, pose.pose.position.y);
	// Set custom mode to OFFBOARD
	mavros_msgs::SetMode first_challenge_set_mode;
	initializeSpace(pose.pose.position.x, pose.pose.position.y);
	first_challenge_set_mode.request.custom_mode = "OFFBOARD";
	// Switch to Offboard mode, arm quadcopter, send service calls every 5 seconds
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();

	while(ros::ok()){
		if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
			if( set_mode_client.call(first_challenge_set_mode) && first_challenge_set_mode.response.mode_sent) {
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		} else {
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
				if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}

		++elapsedTime;
		/*
		if (isPoseAcquired) {
			pose_x = current_pose.pose.position.x;
			pose_y = current_pose.pose.position.y;
		} else {
			pose_x = pose.pose.position.x;
 			pose_y = pose.pose.position.y;
		}
		*/
		pose_x = current_pose.pose.position.x;
		pose_y = current_pose.pose.position.y;		
		updateSpace(pose_x, pose_y);	
		if (moveTo(pose_x, pose_y, elapsedTime)) {
			pose.pose.position.x = pose_x;
			pose.pose.position.y = pose_y;
			std::cout << "moveTo: (" << pose.pose.position.x << ", ";
			std::cout << pose.pose.position.y;
			std::cout << ", " << pose.pose.position.z << ")" << std::endl;
		} else {

		}
		isPoseAcquired = false;
		if (elapsedTime >= DWELL_TIME) {
			elapsedTime = 0;
			std::cout << "main: Restarting elapsedTime" << std::endl;
		}
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}

    return 0;
}

