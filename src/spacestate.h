/*
 * File: spacestate.h
 * Author: Stewart Nash
 * Date: May 31, 2019
 * Version: 0.0.0
 * Description: This contains both the definitions of the space search
 * functions.
 */
#pragma once
#include <vector>

const int WORLD_WIDTH = static_cast<int>((30 + 0.30) / 0.30); // x-dimension
const int WORLD_LENGTH = static_cast<int>((18 + 0.30) / 0.30); // y-dimension
const int WORLD_HEIGHT = 1; // z-dimension
const double X_BUFFER = static_cast<int>(-15.0);
const double Y_BUFFER = static_cast<int>(-11.0);
const double SPACE_FACTOR = 0.30; // conversion from world space grid to coordinates
enum SpaceState {UNKNOWN, SEARCHED, FORBIDDEN, CURRENT};
extern SpaceState worldSpace[WORLD_WIDTH][WORLD_LENGTH];

const unsigned long DWELL_TIME = 75;
const unsigned long PAUSE_TIME = 500;

const int NUMBER_OF_WALLS_SS = 7;
extern std::vector<double> forbiddenPoints[NUMBER_OF_WALLS_SS];

inline int convertLocation(double location);
double distance(int x1, int y1, int x2, int y2);
void initializeSpace(double xLocation, double yLocation);
bool moveTo(double &xLocation, double &yLocation, unsigned long elapsedTime);
void updateSpace(double xLocation, double yLocation);
