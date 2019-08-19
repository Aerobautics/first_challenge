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
#include <opencv2/opencv.hpp>
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

cv::Mat Sightseeing::trackingFunction(cv::Mat inputQuery, cv::Mat inputFrame) {
	cv::Mat output;
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

	//cv::cvtColor(inputQuery, queryImage, cv::COLOR_BGR2GRAY);
	//cv::cvtColor(inputFrame, queryFrame, cv::COLOR_BGR2GRAY);
	queryImage = inputQuery;
	queryFrame = inputFrame;
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

	cv::drawMatches(queryImage, keypoints_1, queryFrame, keypoints_2, matches, output);
	desouza::test_function_01(points_1, points_2);

	return output;
}

cv::Mat Sightseeing::contourFunction(cv::Mat input, double minimumArea, DisplayMode showStats) {
	std::vector<cv::Scalar> colors;

	//Crimson		#DC143C rgb(220, 20, 60)
	//HotPink		#FF69B4 rgb(255, 105, 180)
	//DarkOrange	#FF8C00 rgb(255, 140, 0)
	//Gold			#FFD700 rgb(255, 215, 0)
	//Orchid		#DA70D6 rgb(218, 112, 214)
	//LimeGreen		#32CD32 rgb(50, 205, 50)
	//Turquoise		#40E0D0 rgb(64, 224, 208)
	//Bisque		#FFE4C4 rgb(255, 228, 196)
	//Salmon		#FA8072 rgb(250, 128, 114)
	//OrangeRed		#FF4500 rgb(255, 69, 0)
	//Khaki			#F0E68C rgb(240, 230, 140)
	//SlateBlue		#6A5ACD rgb(106, 90, 205)
	//Olive			#808000 rgb(128, 128, 0)
	//PowderBlue	#B0E0E6 rgb(176, 224, 230)

	colors.push_back(cv::Scalar(220, 20, 60)); // Crimson
	colors.push_back(cv::Scalar(255, 105, 180)); // HotPink
	colors.push_back(cv::Scalar(255, 140, 0)); //DarkOrange
	colors.push_back(cv::Scalar(255, 215, 0)); //Gold
	colors.push_back(cv::Scalar(218, 112, 214)); //Orchid
	colors.push_back(cv::Scalar(50, 205, 50)); //LimeGreen
	colors.push_back(cv::Scalar(64, 224, 208)); //Turquoise
	colors.push_back(cv::Scalar(255, 228, 196)); //Bisque
	colors.push_back(cv::Scalar(250, 128, 114)); //Salmon
	colors.push_back(cv::Scalar(255, 69, 0)); //OrangeRed	
	colors.push_back(cv::Scalar(240, 230, 140)); //Khaki	
	colors.push_back(cv::Scalar(106, 90, 205)); //SlateBlue
	colors.push_back(cv::Scalar(128, 128, 0)); //Olive	
	colors.push_back(cv::Scalar(176, 224, 230)); //PowderBlue

	return contourFunction(input, minimumArea, colors, showStats);
}

cv::Mat Sightseeing::contourFunction(cv::Mat input, double minimumArea, std::vector<cv::Scalar> colors, DisplayMode showStats) {

	cv::Mat output;
	cv::Mat temporary;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Moments> moments;
	std::vector<int> indices;
	std::vector<std::string> stats;
	std::string stat;
	int verticalLocation;
	int uniqueColors;
	double momentArea;
	double contourArea, contourLength;
	double centroidX, centroidY;
	const int VERTICAL_TEXT_OFFSET = 20;
	const int HORIZONTAL_TEXT_OFFSET = 20;

	uniqueColors = colors.size();
	output = input;
	verticalLocation = VERTICAL_TEXT_OFFSET;
	contours = findContours(input, minimumArea);
	for (int i = 0; i < contours.size(); i++) {
		indices.push_back(i);
		moments.push_back(cv::moments(contours[i], true));
	}
	for (int i = 0; i < (int)indices.size(); i++) {
		int j = indices[i];
		momentArea = moments[j].m00;
		contourArea = cv::contourArea(contours[j]);
		contourLength = cv::arcLength(contours[j], true);
		centroidX = moments[j].m10 / momentArea;
		centroidY = moments[j].m01 / momentArea;
		//cv::Scalar color = cv::Scalar(randomNumber.uniform(0, 255), randomNumber.uniform(0, 255), randomNumber.uniform(0, 255));
		cv::Scalar color = colors[i % uniqueColors];
		//cv::drawContours(output, contours, j, cv::Scalar(0, 0, 255), 2, 8);
		cv::drawContours(output, contours, j, color, 2, 8);
		cv::circle(output, cv::Point(centroidX, centroidY), 4, color, -1);
		if (showStats == DisplayMode::STANDARD_OUTPUT) {
			std::cout << "Contour " << j << "(moment area, contour area, contour length): (";
			std::cout << momentArea << ", " << contourArea << ", " << contourLength << ")";
			std::cout << std::endl;
			std::cout << "Contour " << j << "(centroid x, centroid y): (";
			std::cout << centroidX << ", " << centroidY << ")" << std::endl;
		} else if (showStats == DisplayMode::IMAGE) {
			stats.clear();
			stat = "Contour " + std::to_string(j) + "(moment area, contour area, contour length): ";
			stats.push_back(stat);
			stat = "(" + std::to_string(momentArea) + ", " + std::to_string(contourArea) + ", " + std::to_string(contourLength) + ")";
			stats.push_back(stat);
			stat = "Contour " + std::to_string(j) + "(centroid x, centroid y): ";
			stats.push_back(stat);
			stat = "(" + std::to_string(centroidX) + ", " + std::to_string(centroidY) + ")";
			stats.push_back(stat);
			for (int i = 0; i < stats.size(); i++) {
				cv::putText(output, stats[i], cv::Point(HORIZONTAL_TEXT_OFFSET, verticalLocation), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, cv::Scalar(200, 200, 200));
				verticalLocation += VERTICAL_TEXT_OFFSET;
			}
		}
	}

	return output;
}

std::vector<std::vector<cv::Point>> Sightseeing::findContours(cv::Mat input, double minimumArea) {
	cv::Mat grayscale;
	cv::Mat positive, negative;
	cv::Mat combination;
	cv::Mat temporary;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::vector<cv::Point>> output;
	std::vector<cv::Moments> moments;
	std::vector<int> indices;
	double momentArea;
	//double contourArea, contourLength;
	//double centroidX, centroidY;

	if (input.channels() > 1) {
		cv::cvtColor(input, grayscale, cv::COLOR_BGR2GRAY);
	} else {
		grayscale = input;
	}
	cv::threshold(grayscale, positive, 200, 255, cv::THRESH_BINARY);
	cv::threshold(grayscale, negative, 50, 255, cv::THRESH_BINARY_INV);
	cv::add(positive, negative, combination);
	//cv::dilate(combination, combination, cv::Mat(), cv::Point(-1, -1), 2);
	cv::dilate(combination, combination, cv::Mat());
	cv::findContours(combination, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	for (int i = 0; i < contours.size(); i++) {
		indices.push_back(i);
		moments.push_back(cv::moments(contours[i], true));
	}
	for (int i = 0; i < (int)indices.size(); i++) {
		int j = indices[i];
		momentArea = moments[j].m00;
		//contourArea = cv::contourArea(contours[j]);
		//contourLength = cv::arcLength(contours[j], true);
		//centroidX = moments[j].m10 / momentArea;
		//centroidY = moments[j].m01 / momentArea;
		if (momentArea > minimumArea) {
			output.push_back(contours[j]);
		}
	}

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

void sightseeing_test_function_01(std::vector<cv::Point2f> query, std::vector<cv::Point2f> train) {
	desouza::test_function_01(query, train);
}