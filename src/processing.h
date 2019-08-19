/*
 * File: processing.h
 * Author: Aerobotics (Stewart Nash)
 * Description: Image processing for first challenge.
 */
#include <opencv2/opencv.hpp>
#include <vector>

cv::Mat processImage(cv::Mat input) {
	cv::Mat grayscale;
	cv::Mat positive, negative;
	cv::Mat combination;
	cv::Mat output;
	cv::Mat temporary;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<int> indices;

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
	}
	if ((int) indices.size() == 0) {
		cv::cvtColor(combination, output, cv::COLOR_GRAY2BGR);
	} else {
		cv::cvtColor(combination, output, cv::COLOR_GRAY2BGR);
		for (int i = 0; i < (int) indices.size(); i++) {
			int j = indices[i];
			cv::drawContours(output, contours, j, cv::Scalar(0, 0, 255), 2, 8);
		}
	}

	return output;
}
