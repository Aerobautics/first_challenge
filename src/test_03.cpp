#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <time.h>
#include "cellconversion.h"
#include "processing.h"
#include "searchspace.h"

static const int SD_WIDTH = 858;
static const int SD_HEIGHT = 480;
static const std::string FILE_NAME = "groundview.JPG";
//using namespace cv;

int test_function_1();
int test_function_2();

/*
int main(int argc, char* argv[])
{
	int output;

	output = test_function_2();

	return output;
}
*/

int test_function_1() {
	double pose_x, pose_y;

	cv::Mat image;
	image = cv::imread(FILE_NAME);

	if (!image.data) {
		printf("No image data \n");
		return -1;
	}

	cv::namedWindow(PROCESSING_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::namedWindow(SEARCH_WINDOW_NAME, cv::WINDOW_NORMAL);

	// Seed random
	srand(time(NULL));

	for (int i = 0; i < NUMBER_OF_WALLS; i++) {
		forbiddenPoints[i] = getPoints(i);
	}
	initializeSpace(1.0, 1.0);

	for (int i = 0; i < 5; i++) {
		pose_x = static_cast<double>(rand() % 31 - 15);
		pose_y = static_cast<double>(rand() % 19 - 11);
		updateSpace(pose_x, pose_y);
		moveTo(pose_x, pose_y, DWELL_TIME);
		displaySpace();
		processImage(image);
		if (i == 0) {
			cv::resizeWindow("Image Processing", SD_WIDTH, SD_HEIGHT);
			cv::resizeWindow("Search Space", SD_WIDTH, SD_HEIGHT);
			//cv::resizeWindow(PROCESSING_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);
			//cv::resizeWindow(SEARCH_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);
		}
	}

	cv::waitKey(0);
	cv::destroyWindow("Image Processing");
	cv::destroyWindow("Search Space");
	//cv::destroyWindow(PROCESSING_WINDOW_NAME);
	//cv::destroyWindow(SEARCH_WINDOW_NAME);

	return 0;
}

int test_function_2() {
	double pose_x, pose_y;

	cv::Mat image;
	image = cv::imread(FILE_NAME);

	if (!image.data) {
		printf("No image data \n");
		return -1;
	}

	cv::namedWindow(PROCESSING_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::namedWindow(SEARCH_WINDOW_NAME, cv::WINDOW_NORMAL);

	// Seed random
	srand(time(NULL));

	for (int i = 0; i < NUMBER_OF_WALLS; i++) {
		forbiddenPoints[i] = getPoints(i);
	}
	initializeSpace(1.0, 1.0);

	for (int i = 0; i < 5; i++) {
		pose_x = static_cast<double>(rand() % 31 - 15);
		pose_y = static_cast<double>(rand() % 19 - 11);
		updateSpace(pose_x, pose_y);
		newMoveToWaypoint(pose_x, pose_y, DWELL_TIME);
		displaySpace();
		processImage(image);
		if (i == 0) {
			cv::resizeWindow("Image Processing", SD_WIDTH, SD_HEIGHT);
			cv::resizeWindow("Search Space", SD_WIDTH, SD_HEIGHT);
			//cv::resizeWindow(PROCESSING_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);
			//cv::resizeWindow(SEARCH_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);
		}
	}

	cv::waitKey(0);
	cv::destroyWindow("Image Processing");
	cv::destroyWindow("Search Space");
	//cv::destroyWindow(PROCESSING_WINDOW_NAME);
	//cv::destroyWindow(SEARCH_WINDOW_NAME);

	return 0;
}
