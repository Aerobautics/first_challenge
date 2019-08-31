/*
 * Author: Aerobotics (Stewart Nash)
 * File: test_04.cpp
 * Description: Contains testing and debugging functions for
 * image processing.
 */
#include <opencv2\opencv.hpp>
#include "sightseeing.h"
#include <cstdlib>
#include <string>

//#include "spacestate.h"

// Take functionality out of main for multiple methods of use.
//void method_search(int argc, char* argv[]);
//void method_waypoint(int argc, char* argv[]);
//const int FRAME_DELAY = 792;
//const double MINIMUM_AREA = 500;
//enum class DisplayMode { NONE, STANDARD_OUTPUT, IMAGE };

int test_function_3();
int test_function_4();
int test_function_5();
int test_function_6(); // Implement tracking
int test_function_11(); // Implement contour finding
cv::Mat test_function_8(cv::Mat queryImage, cv::Mat frameImage);
cv::Mat test_function_9(cv::Mat inputQuery, cv::Mat inputFrame); // Tracking
cv::Mat test_function_10(cv::Mat input, double minimumArea); // Contour finding
cv::Mat test_function_10(cv::Mat input, double minimumArea, std::vector<cv::Scalar> colors); // Contour finding
cv::Mat test_function_12(cv::Mat input, double minimumArea, DisplayMode showStats = DisplayMode::NONE); // Contour finding calling #13
cv::Mat test_function_12(cv::Mat input, double minimumArea, std::vector<cv::Scalar> colors, DisplayMode showStats = DisplayMode::NONE); // Contour finding calling #13
std::vector<std::vector<cv::Point>> test_function_13(cv::Mat input, double minimumArea); // Return contours with minimum area

/*
int main(int argc, char* argv[])
{
	int output;

	//cv::RNG  randomNumber(static_cast<long int>(time(NULL)));
	output = test_function_6();
	//output = test_function_11();

	system("pause");
	return output;
}
*/

int test_function_3() {
	Sightseeing soothsayer;
	cv::Mat apriltagQuery;
	std::string apriltagQueryPath = "G:\\Ubuntu\\working\\apriltag.jpg";
	//std::string apriltagQueryPath = "G:/Ubuntu/working/apriltag.jpg";
	//std::string apriltagQueryPath = "apriltag.jpg";
	cv::namedWindow("test_function_3", cv::WINDOW_AUTOSIZE);
	apriltagQuery = cv::imread(apriltagQueryPath);
	if (!apriltagQuery.empty()) {
		cv::imshow("test_function_3", apriltagQuery);
		cv::waitKey(0);
	} else {
		std::cout << "[ERROR int test_function_3()]: Cannot read query image.";
		std::cout << std::endl;
		cv::destroyWindow("test_function_3");
		return -1;
	}

	cv::destroyWindow("test_function_3");
	return 0;
}

int test_function_4() {
	Sightseeing soothsayer;
	cv::Mat apriltagFrame;
	std::string apriltagFramePath;
	cv::VideoCapture videoCapture;

	apriltagFramePath = "G:\\Ubuntu\\working\\output_2.avi";
	//apriltagFramePath = "G:/Ubuntu/working/output_2.avi";
	//apriltagFramePath = "output_2.avi";
	videoCapture.open(apriltagFramePath);
	cv::namedWindow("test_function_4", cv::WINDOW_AUTOSIZE);
	do {
		videoCapture >> apriltagFrame;
		if (!apriltagFrame.empty()) {
			cv::imshow("test_function_4", apriltagFrame);
		}
		if (cv::waitKey(FRAME_DELAY) >= 0)
			break;
	} while (!apriltagFrame.empty());

	cv::destroyWindow("test_function_4");
	return 0;
}

int test_function_5() {
	Sightseeing soothsayer;
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
		std::cout << "[ERROR int test_function_5()]: Cannot read query image.";
		return -1;
	}
	cv::namedWindow("test_function_5", cv::WINDOW_AUTOSIZE);
	cv::VideoCapture videoCapture;
	videoCapture.open(apriltagFramePath);
	do {
		videoCapture >> frame;
		if (!frame.empty()) {
			cv::cvtColor(frame, apriltagFrame, cv::COLOR_BGR2GRAY);
			cv::imshow("test_function_5", apriltagFrame);
		} else {

		}
		if (cv::waitKey(FRAME_DELAY) >= 0)
			break;
	} while (!frame.empty());

	cv::destroyWindow("test_function_5");
	return 0;
}

int test_function_6() {
	Sightseeing soothsayer;
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
	cv::namedWindow("test_function_6", cv::WINDOW_AUTOSIZE);
	cv::VideoCapture videoCapture;
	videoCapture.open(apriltagFramePath);
	do {
		videoCapture >> frame;
		if (!frame.empty()) {
			cv::cvtColor(frame, apriltagFrame, cv::COLOR_BGR2GRAY);
			temporary = test_function_9(apriltagQuery, apriltagFrame);
			cv::imshow("test_function_6", temporary);
		} else {

		}
		if (cv::waitKey(FRAME_DELAY) >= 0)
			break;
	} while (!frame.empty());

	cv::destroyWindow("test_function_6");
	return 0;
}

int test_function_11() {
	Sightseeing soothsayer;
	cv::Mat frame;
	cv::Mat apriltagFrame;
	cv::Mat temporary;

	std::string apriltagFramePath = "G:\\Ubuntu\\working\\output_2.avi";
	cv::namedWindow("test_function_11", cv::WINDOW_AUTOSIZE);
	cv::VideoCapture videoCapture;
	videoCapture.open(apriltagFramePath);
	do {
		videoCapture >> frame;
		if (!frame.empty()) {
			temporary = test_function_12(frame, MINIMUM_AREA, DisplayMode::IMAGE);
			cv::imshow("test_function_11", temporary);
		} else {

		}
		if (cv::waitKey(FRAME_DELAY) >= 0)
			break;
	} while (!frame.empty());

	cv::destroyWindow("test_function_11");
	return 0;
}

cv::Mat test_function_8(cv::Mat inputQuery, cv::Mat inputFrame) {
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

	cv::drawMatches(queryImage, keypoints_1, queryFrame, keypoints_2, matches, output);

	return output;
}

// No color
cv::Mat test_function_9(cv::Mat inputQuery, cv::Mat inputFrame) {
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
	sightseeing_test_function_01(points_1, points_2);

	return output;
}

cv::Mat test_function_10(cv::Mat input, double minimumArea) {
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
	
	return test_function_10(input, minimumArea, colors);
}

cv::Mat test_function_10(cv::Mat input, double minimumArea, std::vector<cv::Scalar> colors) {
	cv::Mat grayscale;
	cv::Mat positive, negative;
	cv::Mat combination;
	cv::Mat output;
	cv::Mat temporary;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Moments> moments;
	std::vector<int> indices;
	double momentArea, contourArea, contourLength;
	double centroidX, centroidY;
	int uniqueColors;

	uniqueColors = colors.size();
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
	if ((int)indices.size() == 0) {
		cv::cvtColor(combination, output, cv::COLOR_GRAY2BGR);
	} else {
		cv::cvtColor(combination, output, cv::COLOR_GRAY2BGR);
		for (int i = 0; i < (int)indices.size(); i++) {
			int j = indices[i];
			momentArea = moments[j].m00;
			contourArea = cv::contourArea(contours[j]);
			contourLength = cv::arcLength(contours[j], true);
			centroidX = moments[j].m10 / momentArea;
			centroidY = moments[j].m01 / momentArea;
			if (momentArea > minimumArea) {
				std::cout << "Contour " << j << "(moment area, contour area, contour length): (";
				std::cout << momentArea << ", " << contourArea << ", " << contourLength << ")";
				std::cout << std::endl;
				std::cout << "Contour " << j << "(centroid x, centroid y): (";
				std::cout << centroidX << ", " << centroidY << ")" << std::endl;
			}
			cv::Scalar color = colors[i % uniqueColors];
			cv::drawContours(output, contours, j, color, 2, 8);
			cv::circle(output, cv::Point(centroidX, centroidY), 4, color, -1);
		}
	}

	return output;
}

cv::Mat test_function_12(cv::Mat input, double minimumArea, DisplayMode showStats) {
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

	return test_function_12(input, minimumArea, colors, showStats);
}

cv::Mat test_function_12(cv::Mat input, double minimumArea, std::vector<cv::Scalar> colors, DisplayMode showStats) {

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
	contours = test_function_13(input, minimumArea);
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

std::vector<std::vector<cv::Point>> test_function_13(cv::Mat input, double minimumArea) {
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