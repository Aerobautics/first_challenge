/*
 * File: spacestate.h
 * Author: Stewart Nash
 * Description: This contains both the definitions and implementations of the
 * space search functions. (The implementation should be put in its own cpp
 * file.)
 */
#include <cmath>

const int WORLD_WIDTH = static_cast<int>(30/0.15); // x-dimension
const int WORLD_LENGTH = static_cast<int>(18/0.15); // y-dimension
const int WORLD_HEIGHT = 1; // z-dimension
const double SPACE_FACTOR = 0.15; // conversion from world space grid to coordinates
enum SpaceState {UNKNOWN, SEARCHED, FORBIDDEN, CURRENT};
SpaceState worldSpace[WORLD_WIDTH][WORLD_LENGTH];

const unsigned long DWELL_TIME = 200;
const unsigned long PAUSE_TIME = 500;

inline int convertLocation(double location);
double distance(int x1, int y1, int x2, int y2);
void initializeSpace(double xLocation, double yLocation);
bool moveTo(double &xLocation, double &yLocation, unsigned long elapsedTime);
void updateSpace(double xLocation, double yLocation);

inline int convertLocation(double location) {
	return static_cast<int>(location / SPACE_FACTOR);
}

double distance(int x1, int y1, int x2, int y2) {
        double output;

        output = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
        output = sqrt(output);

        return output;
}

void initializeSpace(double xLocation, double yLocation) {
        int i, j;
        int x, y;

        for (i = 0; i < WORLD_WIDTH; i++) {
                for (j = 0; j < WORLD_LENGTH; j++) {
                        worldSpace[i][j] = SpaceState::UNKNOWN;
                }
        }

        for (i = 0; i < WORLD_WIDTH; i++) {
                for (j = 0; j < WORLD_LENGTH; j++) {
                        x = convertLocation(xLocation);
                        y = convertLocation(yLocation);
                        if (distance(x, y, i, j) < 1.0) {
                                worldSpace[i][j] = SpaceState::CURRENT;
                        }
                }
        }
}

bool moveTo(double &xLocation, double &yLocation, unsigned long elapsedTime) {
        bool output;
        int i, j;
        int x, y;
        double temporary = 1000000.00;
	double temporary_x, temporary_y;

        x = convertLocation(xLocation);
        y = convertLocation(yLocation);
        if (elapsedTime < DWELL_TIME) {
                output = false;
        } else {
                output = true;
                for (i = 0; i < WORLD_WIDTH; i++) {
                        for (j = 0; j < WORLD_LENGTH; j++) {
                                if (distance(x, y, i, j) < temporary) {
                                        if (worldSpace[i][j] == SpaceState::UNKNOWN) {
                                                temporary_x = static_cast<double>(i) * SPACE_FACTOR;
                                                temporary_y = static_cast<double>(j) * SPACE_FACTOR;
                                                temporary = distance(x, y, i, j);
                                        }
                                }
                        }
                }
		xLocation = temporary_x;
		yLocation = temporary_y;
                std::cout << "moveTo: (" << temporary_x << ", " << temporary_y << ")" << std::endl;
        }

        return output;
}


void updateSpace(double xLocation, double yLocation) {
        int i, j;
        int x, y;

        for (i = 0; i < WORLD_WIDTH; i++) {
                for (j = 0; j < WORLD_LENGTH; j++) {
                        if (worldSpace[i][j] == SpaceState::CURRENT) {
                                worldSpace[i][j] == SpaceState::SEARCHED;
                        }
                }
        }

        for (i = 0; i < WORLD_WIDTH; i++) {
                for (j = 0; j < WORLD_LENGTH; j++) {
                        x = convertLocation(xLocation);
                        y = convertLocation(yLocation);
                        if (distance(x, y, i, j) < 1.0) {
                                worldSpace[i][j] = SpaceState::CURRENT;
                        }
                }
        }
}

