/*
 * File: searchspace.h
 * Author: Aerobautics - Stewart Nash
 * Description: Visualizes drone search space
 */
#include <opencv2/opencv.hpp>
#include "spacestate.h"

const int SD_WIDTH_SS = 858;
const int SD_HEIGHT_SS = 480;
static const std::string SEARCH_WINDOW_NAME = "Search Space";
double transformed[WORLD_WIDTH][WORLD_LENGTH];

cv::Mat convertWorldSpace(SpaceState input[WORLD_WIDTH][WORLD_LENGTH]) {
	int i, j;
	cv::Mat output(WORLD_WIDTH, WORLD_LENGTH, CV_64F, transformed);
	double scale;

	scale = 1.0;
	for (i = 0; i < WORLD_WIDTH; i++) {
		for (j = 0; j < WORLD_LENGTH; j++) {
			switch(input[i][j]) {
				case UNKNOWN:
					transformed[i][j] = 0.0 * scale;
					break;
				case SEARCHED:
					transformed[i][j] = 0.5 * scale;
					break;
				case FORBIDDEN:
					transformed[i][j] = 0.75 * scale;
					break;
				case CURRENT:
					transformed[i][j] = 1.0 * scale;
					break;
			}
		}
	}
	cv::transpose(output, output);
	cv::flip(output, output, 1);
	
	return output;	
}

cv::Mat convertWorldSpace() {
	return convertWorldSpace(worldSpace);
}

void displaySpace(std::string windowName, cv::Mat input) {
	cv::imshow(windowName, input);
	cv::waitKey(3);
}

void displaySpace(std::string windowName) {
	displaySpace(windowName, convertWorldSpace());
}

void displaySpace() {
	displaySpace("Search Space");
	//displayProcessing(SEARCH_WINDOW_NAME);
}
