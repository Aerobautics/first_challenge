/*
 * Author: Aerobotics (Stewart Nash)
 * File: first_challenge_imaging.cpp
 * Description: Displays image from iris quadcopter cameras
 *
 */
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

static const std::string WINDOW_NAME = "first_challenge bottom";
static const std::string BOTTOM_WINDOW_NAME = "bottom camera";
static const std::string FRONT_WINDOW_NAME = "front camera";
static const int SD_WIDTH = 858;
static const int SD_HEIGHT = 480;

int frameNumber;

void bottomImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cvImagePtr;

	try {
		cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch(cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::imshow(BOTTOM_WINDOW_NAME, cvImagePtr->image);
	/*
	frameNumber++;
	if (frameNumber % 300) {
		std::cout << "frame number: " << frameNumber << std::endl;
	}
	if (frameNumber > 300000) {
		std::cout << "resetting frame number to 0" << std::endl;
		frameNumber = 0;
	}
	*/
	cv::waitKey(3);
}

void frontImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cvImagePtr;

	try {
		cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch(cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::imshow(FRONT_WINDOW_NAME, cvImagePtr->image);
	cv::waitKey(3);
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "first_challenge_imaging_node");
	ros::NodeHandle nodeHandle;
	ros::Subscriber subscriber;
	image_transport::ImageTransport bottomImageTransport(nodeHandle);
	image_transport::ImageTransport frontImageTransport(nodeHandle);
	image_transport::Subscriber bottomImageSubscriber;
	image_transport::Subscriber frontImageSubscriber;
	image_transport::Publisher imagePublisher;
	cv::namedWindow(BOTTOM_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::namedWindow(FRONT_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::VideoCapture capture;
	cv::Mat frame;

	cv::resizeWindow(BOTTOM_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);
	cv::resizeWindow(FRONT_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);
	frameNumber = 0;
	bottomImageSubscriber = bottomImageTransport.subscribe("/iris_1/camera_down/image_raw", 1, &bottomImageCallback);
	frontImageSubscriber = frontImageTransport.subscribe("/iris_1/camera_forward/image_raw", 1, &frontImageCallback);
	//imagePublisher = imageTransport.advertise("/image_converter/output_video", 1);

	//subscriber = nodeHandle.subscribe<sensor_msgs::Image>("image_raw", 32, &rawImageCallback);
	
	ros::spin();
	cv::destroyWindow(BOTTOM_WINDOW_NAME);
	
	return 0;
}
