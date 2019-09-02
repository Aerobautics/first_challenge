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

const int FRAME_DELAY = 792;
const double MINIMUM_AREA = 500;
enum class ImageProcessor { CONTOUR, TRACKING };
enum class DisplayMode { NONE, STANDARD_OUTPUT, IMAGE };
static const std::string PROCESSING_WINDOW_NAME = "Image Processing";
void displayProcessing(const std::string windowName, cv::Mat input);
void displayProcessing(cv::Mat input);
void sightseeing_test_function_01(std::vector<cv::Point2f> query, std::vector<cv::Point2f> train);

class Sightseeing {
	public:
		Sightseeing(); // Constructor
		std::vector<double> findTags(cv::Mat input);
		std::vector<double> findTags();
		std::vector<double> contourProcessing(cv::Mat input);
		std::vector<double> contourProcessing();
		std::vector<double> trackerProcessing(cv::Mat input);
		std::vector<double> trackerProcessing();
		std::vector<double> defaultProcessing(cv::Mat input);
		void setCurrentImage(cv::Mat input);
		cv::Mat getCurrentImage();
		void setProcessingType(ImageProcessor input);
		ImageProcessor getProcessingType();
		std::vector<int> getImageSize();
		cv::Mat trackingFunction(cv::Mat inputQuery, cv::Mat inputFrame);
		cv::Mat contourFunction(cv::Mat input, double minimumArea, DisplayMode showStats = DisplayMode::NONE);
		cv::Mat contourFunction(cv::Mat input, double minimumArea, std::vector<cv::Scalar> colors, DisplayMode showStats = DisplayMode::NONE);
		cv::Mat contourFunctionGray(cv::Mat input, double minimumArea, std::vector<cv::Scalar> colors, DisplayMode showStats = DisplayMode::NONE);
		std::vector<std::vector<cv::Point>> findContours(cv::Mat input, double minimumArea);
		cv::Mat getOutputImage();
		std::vector<double> getOutputProcessing();
		cv::Mat matchImages(cv::Mat input, cv::Mat reference);
		void setQueryImage(cv::Mat input);
		bool getIsOutputProcessingCurrent();

	private:
		bool isCurrentImageSet;
		bool isQueryImageSet;
		bool isOutputImageCurrent;
		bool isOutputProcessingCurrent;
		bool isUsingOutputImage;
		cv::Mat currentImage;
		cv::Mat queryImage;
		cv::Mat outputImage;
		std::vector<double> outputProcessing;
		ImageProcessor processingType;
		
};

