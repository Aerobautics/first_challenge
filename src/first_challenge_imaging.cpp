#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

static const std::string WINDOW_NAME = "Offboard Imaging";
int frameNumber;

void rawImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cvImagePtr;

	try {
		cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch(cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::imshow(WINDOW_NAME, cvImagePtr->image);
	frameNumber++;
	if (frameNumber % 300) {
		std::cout << "frame number: " << frameNumber << std::endl;
	}
	if (frameNumber > 300000) {
		std::cout << "resetting frame number to 0" << std::endl;
		frameNumber = 0;
	}
	cv::waitKey(3);
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "offboard_imaging_node");
	ros::NodeHandle nodeHandle;
	ros::Subscriber subscriber;
	image_transport::ImageTransport imageTransport(nodeHandle);
	image_transport::Subscriber imageSubscriber;
	image_transport::Publisher imagePublisher;
	cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
	cv::VideoCapture capture;
	cv::Mat frame;

	frameNumber = 0;
	imageSubscriber = imageTransport.subscribe("image_raw", 1, &rawImageCallback);
	//imagePublisher = imageTransport.advertise("/image_converter/output_video", 1);

	//subscriber = nodeHandle.subscribe<sensor_msgs::Image>("image_raw", 32, &rawImageCallback);
	
	ros::spin();
	cv::destroyWindow(WINDOW_NAME);
	
	return 0;
}
