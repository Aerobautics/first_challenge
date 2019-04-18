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
#include <cmath>

const int WORLD_WIDTH = static_cast<int>(30/0.15); // x-dimension
const int WORLD_LENGTH = static_cast<int>(18/0.15); // y-dimension
const int WORLD_HEIGHT = 1; // z-dimension
const double SPACE_FACTOR = 0.15; // conversion from world space grid to coordinates
enum SpaceState {UNKNOWN, SEARCHED, FORBIDDEN, CURRENT};
SpaceState worldSpace[WORLD_WIDTH][WORLD_LENGTH];

enum ImageState {NONE_FOUND, OBSERVE, TARGET, FORCE_OFF};
bool isTargeting = false;
bool isObserving = false;
const unsigned long DWELL_TIME = 200;
const unsigned long OBSERVE_TIME = 500;

mavros_msgs::State current_state;

void initializeSpace(geometry_msgs::PoseStamped input);
void updateSpace(geometry_msgs::PoseStamped input);
double distance(int x1, int y1, int x2, int y2);
geometry_msgs::PoseStamped moveTo(geometry_msgs::PoseStamped input, unsigned long elapsedTime);

/*
 Wall 13
 Wall 15
 Wall 2
 Wall 20
 Wall 3
 Wall 4
 Wall 5
*/

double wallSize[][3] = {
	{20, 0.15, 2.5},
	{10, 0.15, 2.5},
	{30, 0.15, 2.5},
	{10, 0.15, 2.5},
	{18, 0.15, 2.5},
	{30, 0.15, 2.5},
	{18 ,0.15, 2.5}	
};

double wallLocalLocation[][3] = {
	{0, 0, 1.25},
	{0, 0, 1.25},
	{0, 0, 1.25},
	{0, 0, 1.25},
	{0, 0, 1.25},
	{0, 0, 1.25},
	{0, 0, 1.25}
};

double wallLocalAngle[][3] = {
	{0, -0, 0},
	{0, -0, 0},
	{0, -0, 0},
	{0, -0, 0},
	{0, -0, 0},
	{0, -0, 0},
	{0, -0, 0}
};

double wallLocation[][3] = {
	{0, 0, 0},
	{-10, -2, 0},
	{0, -11, 0},
	{10, -2, 0},
	{15, -2, 0},
	{0, 7, 0},
	{-15, -2, 0},
};

double wallAngles[][3] = {
	{0, -0, 0},
	{0, 0, -1.5708},
	{0, -0, 0},
	{0, 0, -1.56904},
	{0, -0, 1.5708},
	{0, -0, 3.14159},
	{0, 0, -1.5708}
};

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "first_challenge_node");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &state_cb);
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	
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
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 2;
	pose.pose.orientation.x = 0;
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = -.99;
	pose.pose.orientation.w = -0.04;
		
	// Send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}

	initializeSpace(pose);

	// Set custom mode to OFFBOARD
	mavros_msgs::SetMode first_challenge_set_mode;
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
		updateSpace(pose);	
		pose = moveTo(pose, elapsedTime);
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

void initializeSpace(geometry_msgs::PoseStamped input) {
	int i, j;
	int x, y;
	double gridResolution = SPACE_FACTOR;
	double xLocation, yLocation;

	xLocation = input.pose.position.x;
	yLocation = input.pose.position.y;
	for (i = 0; i < WORLD_WIDTH; i++) {
		for (j = 0; j < WORLD_LENGTH; j++) {
			worldSpace[i][j] = SpaceState::UNKNOWN;
		}
	}

	for (i = 0; i < WORLD_WIDTH; i++) {
		for (j = 0; j < WORLD_LENGTH; j++) {
			x = xLocation / SPACE_FACTOR;
			y = yLocation / SPACE_FACTOR;
			if (distance(x, y, i, j) < 1.0) {
				worldSpace[i][j] = SpaceState::CURRENT;
			}	
		}
	}
}

void updateSpace(geometry_msgs::PoseStamped input) {
	int i, j;
	int x, y;
	double gridResolution = SPACE_FACTOR;
	double xLocation, yLocation;

	xLocation = input.pose.position.x;
	yLocation = input.pose.position.y;
	for (i = 0; i < WORLD_WIDTH; i++) {
		for (j = 0; j < WORLD_LENGTH; j++) {
			if (worldSpace[i][j] == SpaceState::CURRENT) {
				worldSpace[i][j] == SpaceState::SEARCHED;
			}
		}
	}

	for (i = 0; i < WORLD_WIDTH; i++) {
		for (j = 0; j < WORLD_LENGTH; j++) {
			x = xLocation / SPACE_FACTOR;
			y = yLocation / SPACE_FACTOR;
			if (distance(x, y, i, j) < 1.0) {
				worldSpace[i][j] = SpaceState::CURRENT;
			}	
		}
	}
}

double distance(int x1, int y1, int x2, int y2) {
	double output;

	output = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
	output = sqrt(output);

	return output;
}

geometry_msgs::PoseStamped moveTo(geometry_msgs::PoseStamped input, unsigned long elapsedTime) {
	geometry_msgs::PoseStamped output;
	int i, j;
	int x, y;
	double gridResolution = SPACE_FACTOR;
	double xLocation, yLocation;
	double temporary = 1000000.00;


	xLocation = input.pose.position.x;
	yLocation = input.pose.position.y;
	x = xLocation / SPACE_FACTOR;
	y = yLocation / SPACE_FACTOR;
	if (elapsedTime < DWELL_TIME) {
		output = input;
	} else {
		output = input;
		for (i = 0; i < WORLD_WIDTH; i++) {
			for (j = 0; j < WORLD_LENGTH; j++) {
				if (distance(x, y, i, j) < temporary) {
					if (worldSpace[i][j] == SpaceState::UNKNOWN) {
						output.pose.position.x = static_cast<double>(i) * SPACE_FACTOR;
						output.pose.position.y = static_cast<double>(j) * SPACE_FACTOR;
						temporary = distance(x, y, i, j);
					}
				}	
			}
		}
		std::cout << "moveTo: (" << output.pose.position.x << ", " << output.pose.position.y << ")" << std::endl;
	}

	return output;
} 
