/*
* Author: Aerobotics (Stewart Nash)
* File: test_05.cpp
* Description: Contains testing and debugging functions for
* image processing by the sightseeing class.
*/
#include <opencv2\opencv.hpp>
#include "sightseeing.h"

int test_function_14();
int test_function_15();

int main(int argc, char* argv[])
{
	int output;

	output = test_function_14();
	
	system("pause");
	return output;
}

int test_function_14() {
	Sightseeing soothsayer;
	Sightseeing *sightseeingPointer;
	//Sightseeing &sightseeingReference = soothsayer;
	
	cv::Mat frame;
	cv::Mat apriltagQuery;
	cv::Mat apriltagFrame;
	cv::Mat temporary;

	// Load query image
	std::string apriltagQueryPath = "G:\\Ubuntu\\working\\apriltag.jpg";
	//std::string apriltagQueryPath = "G:/Ubuntu/working/apriltag.jpg";
	//std::string apriltagQueryPath = "apriltag.jpg";
	std::string apriltagFramePath = "G:\\Ubuntu\\working\\output_2.avi";
	temporary = cv::imread(apriltagQueryPath);
	if (!temporary.empty()) {
		cv::cvtColor(temporary, apriltagQuery, cv::COLOR_BGR2GRAY);
	} else {
		std::cout << "[ERROR int test_function_6()]: Cannot read query image.";
		return -1;	
	}
	sightseeingPointer = &soothsayer;
	//sightseeingReference = soothsayer;
	soothsayer.setQueryImage(apriltagQuery);
	//soothsayer.setProcessingType(ImageProcessor::CONTOUR);
	cv::namedWindow("test_function_6", cv::WINDOW_AUTOSIZE);
	cv::VideoCapture videoCapture;
	videoCapture.open(apriltagFramePath);
	do {
		videoCapture >> frame;
		if (!frame.empty()) {
			cv::cvtColor(frame, apriltagFrame, cv::COLOR_BGR2GRAY);
			sightseeingPointer->defaultProcessing(apriltagFrame);
			//sightseeingReference.defaultProcessing(apriltagFrame);
			temporary = sightseeingPointer->getOutputImage();
			cv::imshow("test_function_6", sightseeingPointer->getOutputImage());
			//cv::imshow("test_function_6", sightseeingReference.getOutputImage());
		} else {

		}
		if (cv::waitKey(FRAME_DELAY) >= 0)
			break;
	} while (!frame.empty());

	cv::destroyWindow("test_function_6");
	return 0;
}

int test_function_15() {

	return 0;
}