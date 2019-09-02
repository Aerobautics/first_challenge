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

#include "sightseeing.h"

extern cv::Mat frame;
extern cv::VideoWriter video;
void saveVideo(cv::VideoWriter& output, cv::Mat& input);

static const bool SAVE_VIDEO = false;

static const bool DISPLAY_BOTTOM = false;
static const bool DISPLAY_FRONT = false;
static const bool DISPLAY_PROCESSING = true;
static const bool DISPLAY_SEARCH = false;

static const bool DISPLAY_COORDINATES = false;
static const int PROCESS_DECIMATION = 1; // This is the interval to wait for processing data
static const std::string QUERY_PATH = "/home/stewart/ai_battle_drone/src/first_challenge/src/apriltag.jpg";

class Controller {
	public:
		Controller(); // Constructor
		void searchNavigation(int argc, char *argv[]);
		void waypointNavigation(int argc, char *argv[]);
		void testNavigation(int argc, char *argv[]);
		
	private:
		// TODO: The following members should not be static.
		// TODO: The callback functions should be unlinked from these members.
		static Sightseeing* sightseeingPointer;
		static bool isPoseAcquired;
		static bool isVideoInitialized;
		static int processDecimator;

		static void state_cb(const mavros_msgs::State::ConstPtr& msg);
		static void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
		static void bottomImageCallback(const sensor_msgs::ImageConstPtr& msg);
		static void frontImageCallback(const sensor_msgs::ImageConstPtr& msg);
};

