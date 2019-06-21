/*
 * File: controller.h
 * Author: Stewart Nash (Aerobotics)
 * Date: May 31, 2019
 * Version: 0.0.0
 * Description: This class contains the controlling components 
 * for the drone. This comprises and interface the px4 functions 
 * and mavros library which controls the drones movement. 
 */
#pragma once
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Image.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

extern cv::Mat frame;
extern cv::VideoWriter video;

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void apriltag_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
void bottomImageCallback(const sensor_msgs::ImageConstPtr& msg);
void frontImageCallback(const sensor_msgs::ImageConstPtr& msg);
void saveVideo(cv::VideoWriter& output, cv::Mat& input);

class Controller {
	public:
		Controller(); // Constructor
		void searchNavigation(int argc, char *argv[]);
		void waypointNavigation(int argc, char *argv[]);
};

