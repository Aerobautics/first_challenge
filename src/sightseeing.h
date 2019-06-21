/*
 * File: sightseeing.h
 * Author: Stewart Nash (Aerobotics)
 * Date: May 31, 2019
 * Version: 0.0.0
 * Description: This class contains the computer vision functions
 * for the drone. This includes the image processing functions that 
 * were previoulsy included in 'processing'.
 */
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

enum class ImageProcessor { CONTOUR, TRACKING };
static const std::string PROCESSING_WINDOW_NAME = "Image Processing";
void displayProcessing(const std::string windowName, cv::Mat input);
void displayProcessing(cv::Mat input);

class Sightseeing {
	public:
		Sightseeing(); // Constructor
		std::vector<double> findTags(cv::Mat input);
		std::vector<double> findTags();
		std::vector<double> contourProcessing(cv::Mat input);
		std::vector<double> contourProcessing();
		std::vector<double> trackerProcessing(cv::Mat input);
		std::vector<double> trackerProcessing();
		void setCurrentImage(cv::Mat input);
		cv::Mat getCurrentImage();
		void setProcessingType(ImageProcessor input);
		ImageProcessor getProcessingType();
		std::vector<int> getImageSize();
		void trackingPrototype(cv::Mat inputQuery, cv::Mat inputFrame); // Output relevant data when complete

	private:
		cv::Mat currentImage;
		ImageProcessor processingType;
};

