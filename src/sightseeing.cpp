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
#include "kmeans.h"
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

// Output should return tag centroids when complete
void Sightseeing::trackingPrototype(cv::Mat inputQuery, cv::Mat inputFrame) {
	cv::Mat queryImage;
	cv::Mat queryFrame;
	cv::Ptr<cv::Feature2D> orbDetector;
	std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
	cv::Mat descriptors_1, descriptors_2;
	std::vector<cv::DMatch> matches;
	cv::Ptr<cv::DescriptorMatcher> bruteforceMatcher;
	const int GOOD_MATCH_NUMBER = 7;
	int goodMatchNumber;
	std::vector<cv::Point2f> points_1, points_2;

	cv::cvtColor(inputQuery, queryImage, cv::COLOR_BGR2GRAY);
	cv::cvtColor(inputFrame, queryFrame, cv::COLOR_BGR2GRAY);
	// Initiate ORB detector
	orbDetector = cv::ORB::create();
	 
	// Detect ORB features and compute descriptors
	orbDetector->detectAndCompute(queryImage, cv::Mat(), keypoints_1, descriptors_1);
	orbDetector->detectAndCompute(queryFrame, cv::Mat(), keypoints_2, descriptors_2);

	// Create BFMatcher object
	bruteforceMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

	// Match descriptors or features
	bruteforceMatcher->match(descriptors_1, descriptors_2, matches, cv::Mat());

	// Sort matches by score
	std::sort(matches.begin(), matches.end());
	
	// Remove bad matches
	goodMatchNumber = matches.size() > GOOD_MATCH_NUMBER ? GOOD_MATCH_NUMBER : matches.size();
	matches.erase(matches.begin() + goodMatchNumber, matches.end());

	// Extract location of good matches
	for (int i = 0; i < matches.size(); i++) {
		points_1.push_back(keypoints_1[matches[i].queryIdx].pt);
		points_2.push_back(keypoints_2[matches[i].trainIdx].pt);
	}
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
