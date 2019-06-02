/*
 * File: spacestate.h
 * Author: Aerobotics (Stewart Nash)
 * Date: May 31, 2019
 * Version: 0.0.0
 * Description: This contains both the definitions of the space search
 * functions.
 */
#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

enum StateSpace {UNKNOWN, SEARCHED, FORBIDDEN, CURRENT};

const int WORLD_WIDTH = static_cast<int>((30 + 0.30) / 0.30); // x-dimension
const int WORLD_LENGTH = static_cast<int>((18 + 0.30) / 0.30); // y-dimension
const int WORLD_HEIGHT = 1; // z-dimension
const double X_BUFFER = static_cast<int>(-15.0);
const double Y_BUFFER = static_cast<int>(-11.0);
const double SPACE_FACTOR = 0.30; // conversion from world space grid to coordinates
const unsigned long DWELL_TIME = 75;
const unsigned long PAUSE_TIME = 500;
const int NUMBER_OF_WALLS_SS = 7;
const int SD_WIDTH_SS = 858;
const int SD_HEIGHT_SS = 480;

static const std::string SEARCH_WINDOW_NAME = "Search Space";

extern StateSpace worldSpace[WORLD_WIDTH][WORLD_LENGTH];
extern std::vector<double> forbiddenPoints[NUMBER_OF_WALLS_SS];
extern double transformed[WORLD_WIDTH][WORLD_LENGTH];

class SpaceState {
	public:
		SpaceState();
		bool moveTo(double &xLocation, double &yLocation, unsigned long elapsedTime);
		void updateSpace(double xLocation, double yLocation);
		void initializeSpace(double xLocation, double yLocation);

		// Display search spaced
		void displaySpace(std::string windowName, cv::Mat input);
		void displaySpace(std::string windowName);
		void displaySpace();

	private:
		//inline int convertLocation(double location);
		int convertLocation(double location);
		int xConvertPoint(double xLocation);
		int yConvertPoint(double yLocation);
		double xConvertCell(int xCell);
		double yConvertCell(int yCell);
		double distance(int x1, int y1, int x2, int y2);

		// Display search spaced
		cv::Mat convertWorldSpace(StateSpace input[WORLD_WIDTH][WORLD_LENGTH]);
		cv::Mat convertWorldSpace();
};
