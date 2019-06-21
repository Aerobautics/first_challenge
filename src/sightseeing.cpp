/*
 * File: sightseeing.cpp
 * Author: Stewart Nash (Aerobotics)
 * Date: May 31, 2019
 * Version: 0.0.0
 * Description: This class contains the computer vision functions
 * for the drone. This includes the image processing functions that 
 * were previoulsy included in 'processing'.
 */
#include "sightseeing.h"
#include <vector>
#include <iostream>

Sightseeing::Sightseeing() {
	processingType = ImageProcessor::CONTOUR;
}

std::vector<double> Sightseeing::findTags(cv::Mat input) {
	std::vector<double> output;

	switch (processingType) {
		case ImageProcessor::CONTOUR:
			output = contourProcessing(input);
			break;
		case ImageProcessor::TRACKING:
			output = trackerProcessing(input);
			break;
		default:
			output = contourProcessing(input);
			std::cout << "[ERROR std::vector Sightseeing::findTags(cv::Mat)]: Unrecognized \'ImageProcess processingType\'. Default to CONTOUR.";
			std::cout << std::endl;
			break;
	}

	return output;
}

std::vector<double> Sightseeing::findTags() {
	return findTags(currentImage);
}

std::vector<double> Sightseeing::contourProcessing(cv::Mat input) {
	std::vector<double> output;

	return output;
}

std::vector<double> Sightseeing::contourProcessing() {
	return contourProcessing(currentImage);
}

std::vector<double> Sightseeing::trackerProcessing(cv::Mat input) {
	std::vector<double> output;

	return output;
}

std::vector<double> Sightseeing::trackerProcessing() {
	return trackerProcessing(currentImage);
}

void Sightseeing::setCurrentImage(cv::Mat input) {
	currentImage = input;
}

cv::Mat Sightseeing::getCurrentImage() {
	// Make a copy to return?
	return currentImage;
}

void Sightseeing::setProcessingType(ImageProcessor input) {
	processingType = input;
}

ImageProcessor Sightseeing::getProcessingType() {
	return processingType;
}

std::vector<int> Sightseeing::getImageSize() {
	std::vector<int> output;

	return output;
}

// Display methods previously in "processing.h"

void displayProcessing(const std::string windowName, cv::Mat input) {
	cv::imshow(windowName, input);
	cv::waitKey(3);
}

void displayProcessing(cv::Mat input) {
	displayProcessing("Image Processing", input);
	//displayProcessing(PROCESSING_WINDOW_NAME, input);
}
