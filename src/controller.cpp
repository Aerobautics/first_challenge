/*
 * File: controller.cpp
 * Author: Stewart Nash (Aerobotics)
 * Date: May 31, 2019
 * Version: 0.0.0
 * Description: This class contains the controlling components 
 * for the drone. This comprises and interface the px4 functions 
 * and mavros library which controls the drones movement. 
 */
#include "controller.h"

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#include <iostream>

//#include "cellconversion.h"
#include "navigation.h"
#include "spacestate.h"
#include "waypoint.h"

// Remove the following variables
// Found in cellconversion.h
static const int NUMBER_OF_WALLS_STATIC  = 7;

static const std::string WINDOW_NAME = "first_challenge bottom";
static const std::string BOTTOM_WINDOW_NAME = "bottom camera";
static const std::string FRONT_WINDOW_NAME = "front camera";

static const int SD_WIDTH = 858;
static const int SD_HEIGHT = 480;
static const int HD_WIDTH = 1280;
static const int HD_HEIGHT = 720;
static const int FHD_WIDTH = 1920;
static const int FHD_HEIGHT = 1080;
static const int X_OFFSET = 10;
static const int Y_OFFSET = -9;

static const double X_SCALE = 1.00;
static const double Y_SCALE = 1.00;

int video_width = HD_WIDTH;
int video_height = HD_HEIGHT;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

cv::Mat frame;
cv::VideoWriter video;

bool Controller::isVideoInitialized;
bool Controller::isPoseAcquired;
int Controller::processDecimator;
Sightseeing* Controller::sightseeingPointer;

Controller::Controller() {
	isVideoInitialized = false;
	processDecimator = 0;
}

void Controller::searchNavigation(int argc, char *argv[]) {
	Sightseeing soothsayer;

	SpaceState spaceState;
	isPoseAcquired = false;
	ros::init(argc, argv, "first_challenge_node");
	ros::NodeHandle nodeHandle;

	sightseeingPointer = &soothsayer;	
	ros::Subscriber state_sub = nodeHandle.subscribe<mavros_msgs::State>("/mavros/state", 10, &state_cb);
	ros::Subscriber local_pos_sub = nodeHandle.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &local_pos_cb);
	ros::Publisher local_pos_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
	ros::ServiceClient arming_client = nodeHandle.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nodeHandle.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

	image_transport::ImageTransport bottomImageTransport(nodeHandle);
	image_transport::ImageTransport frontImageTransport(nodeHandle);
	image_transport::Subscriber bottomImageSubscriber;
	image_transport::Subscriber frontImageSubscriber;
	image_transport::Publisher imagePublisher;

	bottomImageSubscriber = bottomImageTransport.subscribe("/iris_1/camera_down/image_raw", 1, &bottomImageCallback);
	frontImageSubscriber = frontImageTransport.subscribe("/iris_1/camera_forward/image_raw", 1, &frontImageCallback);
	//imagePublisher = imageTransport.advertise("/image_converter/output_video", 1);
	//subscriber = nodeHandle.subscribe<sensor_msgs::Image>("image_raw", 32, &rawImageCallback);

	cv::VideoCapture capture;
	
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

	//image = cv::imread(FILE_NAME);
	//if (!image.data) {
	//	printf("No image data \n");
	//	return -1;
	//}

	if (DISPLAY_BOTTOM) {
		cv::namedWindow(BOTTOM_WINDOW_NAME, cv::WINDOW_NORMAL);
		cv::resizeWindow(BOTTOM_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);		
	}
	if (DISPLAY_FRONT) {
		cv::namedWindow(FRONT_WINDOW_NAME, cv::WINDOW_NORMAL);
		cv::resizeWindow(FRONT_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);		
	}
	if (DISPLAY_SEARCH) {
		cv::namedWindow(SEARCH_WINDOW_NAME, cv::WINDOW_NORMAL);
		//cv::resizeWindow(SEARCH_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);
		cv::resizeWindow("Search Space", SD_WIDTH, SD_HEIGHT);		
	}
	
	if (DISPLAY_COORDINATES) {
		std::cout << "Starting position (" << current_pose.pose.position.x << ", ";
		std::cout << current_pose.pose.position.y << ")" << std::endl;		
	}
	/*
	pose.pose.position.x = current_pose.pose.position.x; // 0?
	pose.pose.position.y = current_pose.pose.position.y; // 0?
	pose.pose.position.z = current_pose.pose.position.z; // 2?
	pose.pose.orientation.x = current_pose.pose.orientation.x; // 0?
	pose.pose.orientation.y = current_pose.pose.orientation.y; // 0?
	pose.pose.orientation.z = current_pose.pose.orientation.z; // -0.99?
	pose.pose.orientation.w = -current_pose.pose.orientation.w; // -0.04?
	*/
	pose.pose.position.x = 0; // 0?
	pose.pose.position.y = 0; // 0?
	pose.pose.position.z = 2; // 2?
	pose.pose.orientation.x = 0; // 0?
	pose.pose.orientation.y = 0; // 0?
	pose.pose.orientation.z = -0.99; // -0.99?
	pose.pose.orientation.w = -0.04; // -0.04?
	pose.pose.position.z = 2;		
	// Send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}
	// Initialize spacestate values -> move this to header
	for (int i = 0; i < NUMBER_OF_WALLS_STATIC; i++) {
		forbiddenPoints[i] = navigationGetPoints(i);
	}
	spaceState.initializeSpace(pose.pose.position.x + X_OFFSET, pose.pose.position.y + Y_OFFSET);

	// Set custom mode to OFFBOARD
	mavros_msgs::SetMode first_challenge_set_mode;
	first_challenge_set_mode.request.custom_mode = "OFFBOARD";
	// Switch to Offboard mode, arm quadcopter, send service calls every 5 seconds
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();

	if (SAVE_VIDEO) {
		video = cv::VideoWriter("output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(video_width, video_height)); 
	}

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
		pose_x = current_pose.pose.position.x + X_OFFSET;
		pose_y = current_pose.pose.position.y + Y_OFFSET;		
		spaceState.updateSpace(pose_x, pose_y);	
		if (spaceState.moveTo(pose_x, pose_y, elapsedTime)) {
			pose.pose.position.x = pose_x - X_OFFSET;
			pose.pose.position.y = pose_y - Y_OFFSET;
			if (DISPLAY_COORDINATES) {
				std::cout << "moveTo: (" << pose.pose.position.x + X_OFFSET;
				std::cout << ", " << pose.pose.position.y + Y_OFFSET;
				std::cout << ", " << pose.pose.position.z << ")" << std::endl;				
			}
			elapsedTime = 0;
		} else {

		}
		spaceState.displaySpace();
		//processImage(frame);
		soothsayer.findTags(frame);
		isPoseAcquired = false;
		//if (elapsedTime >= DWELL_TIME) {
		//	elapsedTime = 0;
		//	std::cout << "main: Restarting elapsedTime" << std::endl;
		//}
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}

	
	if (DISPLAY_BOTTOM) {
		cv::destroyWindow(BOTTOM_WINDOW_NAME);
	}
	if (DISPLAY_FRONT) {
		cv::destroyWindow(FRONT_WINDOW_NAME);		
	}
	if (DISPLAY_SEARCH) {
		//cv::destroyWindow(SEARCH_WINDOW_NAME);
		cv::destroyWindow("Search Space");
	}		

	if (SAVE_VIDEO) {
		video.release();
	}
}

void Controller::waypointNavigation(int argc, char *argv[]) {
	Waypoint waypoint;
	SpaceState spaceState;
	Sightseeing soothsayer;
	isPoseAcquired = false;
	ros::init(argc, argv, "first_challenge_node");
	ros::NodeHandle nodeHandle;

	sightseeingPointer = &soothsayer;
	ros::Subscriber state_sub = nodeHandle.subscribe<mavros_msgs::State>("/mavros/state", 10, &state_cb);
	ros::Subscriber local_pos_sub = nodeHandle.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &local_pos_cb);
	ros::Publisher local_pos_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
	ros::ServiceClient arming_client = nodeHandle.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nodeHandle.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

	image_transport::ImageTransport bottomImageTransport(nodeHandle);
	image_transport::ImageTransport frontImageTransport(nodeHandle);
	image_transport::Subscriber bottomImageSubscriber;
	image_transport::Subscriber frontImageSubscriber;
	image_transport::Publisher imagePublisher;

	bottomImageSubscriber = bottomImageTransport.subscribe("/iris_1/camera_down/image_raw", 1, &bottomImageCallback);
	frontImageSubscriber = frontImageTransport.subscribe("/iris_1/camera_forward/image_raw", 1, &frontImageCallback);

	cv::VideoCapture capture;
	
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

	if (DISPLAY_BOTTOM) {
		cv::namedWindow(BOTTOM_WINDOW_NAME, cv::WINDOW_NORMAL);
		cv::resizeWindow(BOTTOM_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);		
	}
	if (DISPLAY_FRONT) {
		cv::namedWindow(FRONT_WINDOW_NAME, cv::WINDOW_NORMAL);
		cv::resizeWindow(FRONT_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);		
	}

	if (DISPLAY_COORDINATES) {
		std::cout << "Starting position (" << current_pose.pose.position.x << ", ";
		std::cout << current_pose.pose.position.y << ")" << std::endl;		
	}

	pose.pose.position.x = 0; // 0?
	pose.pose.position.y = 0; // 0?
	pose.pose.position.z = 2; // 2?
	pose.pose.orientation.x = 0; // 0?
	pose.pose.orientation.y = 0; // 0?
	pose.pose.orientation.z = -0.99; // -0.99?
	pose.pose.orientation.w = -0.04; // -0.04?
	pose.pose.position.z = 2;		
	// Send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}
	// Initialize spacestate values -> move this to header
	for (int i = 0; i < NUMBER_OF_WALLS_STATIC; i++) {
		forbiddenPoints[i] = navigationGetPoints(i);
	}
	spaceState.initializeSpace(pose.pose.position.x / X_SCALE + X_OFFSET, pose.pose.position.y / Y_SCALE + Y_OFFSET);

	// Set custom mode to OFFBOARD
	mavros_msgs::SetMode first_challenge_set_mode;
	first_challenge_set_mode.request.custom_mode = "OFFBOARD";
	// Switch to Offboard mode, arm quadcopter, send service calls every 5 seconds
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();

	if (SAVE_VIDEO) {
		video = cv::VideoWriter("output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(video_width, video_height));
	}
 
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

		pose_x = current_pose.pose.position.x / X_SCALE + X_OFFSET;
		pose_y = current_pose.pose.position.y / Y_SCALE + Y_OFFSET;		
		spaceState.updateSpace(pose_x, pose_y);	
		if (waypoint.moveToWaypoint(pose_x, pose_y, elapsedTime)) {
			pose.pose.position.x = pose_x - X_OFFSET;
			pose.pose.position.y = pose_y - Y_OFFSET;
			pose.pose.position.x = X_SCALE * (pose_x - X_OFFSET);
			pose.pose.position.y = Y_SCALE * (pose_y - Y_OFFSET);
			if (DISPLAY_COORDINATES) {
				std::cout << "moveTo: (" << pose.pose.position.x / X_SCALE + X_OFFSET;
				std::cout << ", " << pose.pose.position.y / Y_SCALE + Y_OFFSET;
				std::cout << ", " << pose.pose.position.z << ")" << std::endl;				
			}
			elapsedTime = 0;
		} else {
			++elapsedTime;
		}
		//spaceState.displaySpace();
		//processImage(frame);
		//soothsayer.findTags(frame);
		isPoseAcquired = false;

		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}

	if (DISPLAY_BOTTOM) {
		cv::destroyWindow(BOTTOM_WINDOW_NAME);
	}
	if (DISPLAY_FRONT) {
		cv::destroyWindow(FRONT_WINDOW_NAME);
	}

	if (SAVE_VIDEO) {
		video.release();
	}
}

void Controller::testNavigation(int argc, char *argv[]) {
	Waypoint waypoint;
	SpaceState spaceState;
	Sightseeing soothsayer;
	isPoseAcquired = false;
	ros::init(argc, argv, "first_challenge_node");
	ros::NodeHandle nodeHandle;

	sightseeingPointer = &soothsayer;
	ros::Subscriber state_sub = nodeHandle.subscribe<mavros_msgs::State>("/mavros/state", 10, &state_cb);
	ros::Subscriber local_pos_sub = nodeHandle.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &local_pos_cb);
	ros::Publisher local_pos_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
	ros::ServiceClient arming_client = nodeHandle.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nodeHandle.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

	image_transport::ImageTransport bottomImageTransport(nodeHandle);
	image_transport::ImageTransport frontImageTransport(nodeHandle);
	image_transport::Subscriber bottomImageSubscriber;
	image_transport::Subscriber frontImageSubscriber;
	image_transport::Publisher imagePublisher;
	
	cv::Mat temporary;
	cv::Mat queryImage;
	
	soothsayer.setProcessingType(ImageProcessor::CONTOUR);
	temporary = cv::imread(QUERY_PATH);
	cv::cvtColor(temporary, queryImage, cv::COLOR_BGR2GRAY);
	soothsayer.setQueryImage(queryImage);

	bottomImageSubscriber = bottomImageTransport.subscribe("/iris_1/camera_down/image_raw", 1, &bottomImageCallback);
	frontImageSubscriber = frontImageTransport.subscribe("/iris_1/camera_forward/image_raw", 1, &frontImageCallback);

	cv::VideoCapture capture;
	
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
	
	if (DISPLAY_PROCESSING) {
		cv::namedWindow(PROCESSING_WINDOW_NAME, cv::WINDOW_NORMAL);
		//cv::resizeWindow(PROCESSING_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);
		cv::resizeWindow("Image Processing", SD_WIDTH, SD_HEIGHT);		
	}

	if (DISPLAY_COORDINATES) {
		std::cout << "Starting position (" << current_pose.pose.position.x << ", ";
		std::cout << current_pose.pose.position.y << ")" << std::endl;		
	}

	pose.pose.position.x = 0; // 0?
	pose.pose.position.y = 0; // 0?
	pose.pose.position.z = 2; // 2?
	pose.pose.orientation.x = 0; // 0?
	pose.pose.orientation.y = 0; // 0?
	pose.pose.orientation.z = -0.99; // -0.99?
	pose.pose.orientation.w = -0.04; // -0.04?
	pose.pose.position.z = 2;		
	// Send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}
	// Initialize spacestate values -> move this to header
	for (int i = 0; i < NUMBER_OF_WALLS_STATIC; i++) {
		forbiddenPoints[i] = navigationGetPoints(i);
	}
	spaceState.initializeSpace(pose.pose.position.x / X_SCALE + X_OFFSET, pose.pose.position.y / Y_SCALE + Y_OFFSET);

	// Set custom mode to OFFBOARD
	mavros_msgs::SetMode first_challenge_set_mode;
	first_challenge_set_mode.request.custom_mode = "OFFBOARD";
	// Switch to Offboard mode, arm quadcopter, send service calls every 5 seconds
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();

	if (SAVE_VIDEO) {
		video = cv::VideoWriter("output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(video_width, video_height));
	}
 
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

		pose_x = current_pose.pose.position.x / X_SCALE + X_OFFSET;
		pose_y = current_pose.pose.position.y / Y_SCALE + Y_OFFSET;		
		spaceState.updateSpace(pose_x, pose_y);	
		if (waypoint.moveToWaypoint(pose_x, pose_y, elapsedTime)) {
			pose.pose.position.x = pose_x - X_OFFSET;
			pose.pose.position.y = pose_y - Y_OFFSET;
			pose.pose.position.x = X_SCALE * (pose_x - X_OFFSET);
			pose.pose.position.y = Y_SCALE * (pose_y - Y_OFFSET);
			if (DISPLAY_COORDINATES) {
				std::cout << "moveTo: (" << pose.pose.position.x / X_SCALE + X_OFFSET;
				std::cout << ", " << pose.pose.position.y / Y_SCALE + Y_OFFSET;
				std::cout << ", " << pose.pose.position.z << ")" << std::endl;				
			}
			elapsedTime = 0;
		} else {
			++elapsedTime;
		}
		if (DISPLAY_SEARCH) {
			spaceState.displaySpace();			
		}
		//processImage(frame);
		//soothsayer.findTags(frame);
		isPoseAcquired = false;

		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}

	if (DISPLAY_PROCESSING) {
		//cv::destroyWindow(PROCESSING_WINDOW_NAME);
		cv::destroyWindow("Image Processing");	
	}

	if (SAVE_VIDEO) {
		video.release();
	}
}

void Controller::state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

void Controller::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	current_pose = *msg;
	isPoseAcquired = true;
}

void Controller::bottomImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cvImagePtr;
	cv::Mat processingFrame;

	try {
		cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch(cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	if (DISPLAY_BOTTOM) {
		cv::imshow(BOTTOM_WINDOW_NAME, cvImagePtr->image);		
	}
	if (DISPLAY_PROCESSING) {
		if (sightseeingPointer != nullptr) {
			if (processDecimator % PROCESS_DECIMATION == 0) {
				cv::cvtColor(cvImagePtr->image, processingFrame, cv::COLOR_BGR2GRAY);
				sightseeingPointer->defaultProcessing(processingFrame);
				cv::imshow(PROCESSING_WINDOW_NAME, sightseeingPointer->getOutputImage());			
			} else {
					
			}
		} else {
			
		}
		++processDecimator;
	} else {
		
	}
	cvImagePtr->image.copyTo(frame);
	if (!isVideoInitialized) {
		cv::Size frameSize = frame.size();
		video_width = frameSize.width;
		video_height = frameSize.height;
		isVideoInitialized = true;
	} else {
		if (SAVE_VIDEO) {
			saveVideo(video, frame);
		}
	}
	cv::waitKey(3);
}

void Controller::frontImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cvImagePtr;

	try {
		cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch(cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	if (DISPLAY_FRONT) {
		cv::imshow(FRONT_WINDOW_NAME, cvImagePtr->image);		
	}
	cv::waitKey(3);
}

void saveVideo(cv::VideoWriter &output, cv::Mat& input) {
	cv::Mat colorFrame;
	cv::Mat temporary;

	cv::cvtColor(input, temporary, cv::COLOR_BGR2GRAY);
	cv::cvtColor(temporary, colorFrame, cv::COLOR_GRAY2BGR);		
	output.write(colorFrame);
}
