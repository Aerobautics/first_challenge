/*
 * File: spacestate.h
 * Author: Stewart Nash
 * Description: This contains both the definitions and implementations of the
 * space search functions. (The implementation should be put in its own cpp
 * file.)
 */
#include <cmath>
#include <vector>

const int WORLD_WIDTH = static_cast<int>((30 + 0.30) / 0.30); // x-dimension
const int WORLD_LENGTH = static_cast<int>((18 + 0.30) / 0.30); // y-dimension
const int WORLD_HEIGHT = 1; // z-dimension
const double X_BUFFER = static_cast<int>(-15.0);
const double Y_BUFFER = static_cast<int>(-11.0);
const double SPACE_FACTOR = 0.30; // conversion from world space grid to coordinates
enum SpaceState {UNKNOWN, SEARCHED, FORBIDDEN, CURRENT};
SpaceState worldSpace[WORLD_WIDTH][WORLD_LENGTH];

const unsigned long DWELL_TIME = 75;
const unsigned long PAUSE_TIME = 500;

const int NUMBER_OF_WALLS_SS = 7;
std::vector<double> forbiddenPoints[NUMBER_OF_WALLS_SS];

inline int convertLocation(double location);
double distance(int x1, int y1, int x2, int y2);
void initializeSpace(double xLocation, double yLocation);
bool moveTo(double &xLocation, double &yLocation, unsigned long elapsedTime);
void updateSpace(double xLocation, double yLocation);

inline int convertLocation(double location) {
	return static_cast<int>(location / SPACE_FACTOR);
}

inline int xConvertPoint(double xLocation) {
	return static_cast<int>((xLocation - X_BUFFER) / SPACE_FACTOR);
}

inline int yConvertPoint(double yLocation) {
	return static_cast<int>((yLocation - Y_BUFFER) / SPACE_FACTOR);
}

inline double xConvertCell(int xCell) {
	return static_cast<double>(xCell) * SPACE_FACTOR + X_BUFFER;
}

inline double yConvertCell(int yCell) {
	return static_cast<double>(yCell) * SPACE_FACTOR + Y_BUFFER;
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
	int count;
	
        for (i = 0; i < WORLD_WIDTH; i++) {
                for (j = 0; j < WORLD_LENGTH; j++) {
                        worldSpace[i][j] = UNKNOWN;
                }
        }

        for (i = 0; i < WORLD_WIDTH; i++) {
                for (j = 0; j < WORLD_LENGTH; j++) {
                        x = xConvertPoint(xLocation);
                        y = yConvertPoint(yLocation);
                        if (distance(x, y, i, j) < 1.0001) {
                                worldSpace[i][j] = CURRENT;
                        }
                }
        }

	for (j = 0; j < NUMBER_OF_WALLS_SS; j++) {
		//std::cout << std::endl << "Wall " << j << std::endl;
		count = (forbiddenPoints[j]).size() / 2;
		for (i = 0; i < count; i++) {
			x = xConvertPoint((forbiddenPoints[j])[2 * i]);
			y = yConvertPoint((forbiddenPoints[j])[2 * i + 1]);
			//x = xConvertPoint((forbiddenPoints[j]).at(2 * i));
			//y = yConvertPoint((forbiddenPoints[j]).at(2 * i + 1));
			worldSpace[x][y] = FORBIDDEN;
			//std::cout << "(" << x << ", " << y << ") ";
			//if (i > 0 && i % 4 == 0) {
			//	std::cout << std::endl;
			//}
		} 
	}
	// Insert buffer zone
	for (j = 0; j < NUMBER_OF_WALLS_SS; j++) {
		count = (forbiddenPoints[j]).size() / 2;
		for (i = 0; i < count; i++) {
			x = xConvertPoint((forbiddenPoints[j])[2 * i]);
			y = yConvertPoint((forbiddenPoints[j])[2 * i + 1]);
			// Buffer zone
			for (int m = 0; m < WORLD_WIDTH; m++) {
				for (int n = 0; n < WORLD_LENGTH; n++) {
					if (distance(x, y, i, j) < sqrt(2.001)) {
						worldSpace[m][n] = FORBIDDEN;
					}					
				}
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

        x = xConvertPoint(xLocation);
        y = yConvertPoint(yLocation);
        if (elapsedTime < DWELL_TIME) {
                output = false;
        } else {
                output = true;
                for (i = 0; i < WORLD_WIDTH; i++) {
                        for (j = 0; j < WORLD_LENGTH; j++) {
                                if (distance(x, y, i, j) < temporary) {
                                        if (worldSpace[i][j] == UNKNOWN) {
						temporary_x = xConvertCell(i);
						temporary_y = yConvertCell(j);
                                                temporary = distance(x, y, i, j);
                                        }
                                }
                        }
                }
		xLocation = temporary_x;
		yLocation = temporary_y;
        }

        return output;
}


void updateSpace(double xLocation, double yLocation) {
        int i, j;
        int x, y;

        for (i = 0; i < WORLD_WIDTH; i++) {
                for (j = 0; j < WORLD_LENGTH; j++) {
                        if (worldSpace[i][j] == CURRENT) {
                                worldSpace[i][j] = SEARCHED;
                        }
                }
        }

        for (i = 0; i < WORLD_WIDTH; i++) {
                for (j = 0; j < WORLD_LENGTH; j++) {
                        x = xConvertPoint(xLocation);
                        y = yConvertPoint(yLocation);
                        if (distance(x, y, i, j) < 1.0) {
                                worldSpace[i][j] = CURRENT;
                        }
                }
        }
}

//----------------------------------Waypoint Code-----------------------------------------------
const int WAYPOINT_NUMBER = 54;
const unsigned long WAYPOINT_DWELL_TIME = 500;
bool isFirstPoint = true;
int currentWaypoint = 0;

double waypoints[WAYPOINT_NUMBER][2] = { // Size of 54x2
	{14,-10.5},
	{9.3,-10.5},
	{9.1,-0.5},
	{-9.0,-0.5},
	{-9.0,-2},
	{8,-2},
	{8 ,-3.5},
	{-9,-3.5},
	{-9,-5},
	{8,-5},
	{8,-10.5},
	{6.5,-10.5},
	{6.5, -6.5},
	{5,-6.5},
	{5,-10.5},
	{3.5,-10.5},
	{3.5,-6.5},
	{2,-6.5},
	{2,-10.5},
	{0.5,-10.5},
	{0.5,-6.5},
	{-1,-6.5},
	{-1,-10.5},
	{-2.5,-10.5},
	{-2.5,-6.5},
	{-4,-6.5},
	{-4,-10.5},
	{-5.5,-10.5},
	{-5.5,-6.5},
	{-7,-6.5},
	{-7,-10.5},
	{-8.5,-6.5},
	{-8.5,-10.5},
	{-14.5,-10.5},
	{-14.5,6},
	{14.5,6},
	{14.5,-9},
	{13,-9},
	{13,4.5},
	{-13,4.5},
	{-13,-9},
	{-11,-9},
	{-11,4.5},
	{-9,4.5},
	{-9,1},
	{9,1},
	{9,2.5},
	{-8,2.5},
	{-8,3.5},
	{8,3.5},
	{11,3.5},
	{11,-9.5},
	{12,-9.5},
	{12,4}
};

bool moveToWaypoint(double &xLocation, double &yLocation, unsigned long elapsedTime) {
        bool output;

        if (elapsedTime < WAYPOINT_DWELL_TIME) {
                output = false;
		if (isFirstPoint) {
			xLocation = waypoints[currentWaypoint][0];
			yLocation = waypoints[currentWaypoint][1];
			isFirstPoint = false;
		}
        } else {
                output = true;
		if (currentWaypoint < WAYPOINT_NUMBER) {
			++currentWaypoint;
		} else { // Reset waypoint to restart search
			currentWaypoint = 0;
		}
		xLocation = waypoints[currentWaypoint][0];
		yLocation = waypoints[currentWaypoint][1];
        }

        return output;
}

