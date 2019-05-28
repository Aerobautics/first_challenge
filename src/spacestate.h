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
const int NEW_WAYPOINT_NUMBER = 55;
const int DELAY_NUMBER = 15;
const unsigned long WAYPOINT_DWELL_TIME = 15;
const unsigned long WAYPOINT_PAUSE_TIME = 250;
const double POINTS_PER_DISTANCE = 3.0;
bool isFirstPoint = true;
bool isPathCreated = false;
int currentWaypoint = 0;
std::vector<double> travelPath[2];

bool moveToWaypoint(double &xLocation, double &yLocation, unsigned long elapsedTime);
int intermediatePoints(double x1, double y1, double x2, double y2);
double waypointDistance(double x1, double y1, double x2, double y2);
double waypointDistanceX(double x1, double x2);
double waypointDistanceY(double y1, double y2);
double waypointLength(double x1, double y1, double x2, double y2);
double waypointLengthX(double x1, double y1, double x2, double y2);
double waypointLengthY(double x1, double y1, double x2, double y2);
void createTravelPath();
bool newMoveToWaypoint(double &xLocation, double &yLocation, unsigned long elapsedTime);

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

double newWaypoints[NEW_WAYPOINT_NUMBER][2] = { // Size of 54x2
	{13.5,-10.0},
	{9.3,-10.0},
	{8.9,-1.0},
	{-8.5,-1.5},
	{-8.5,-2},
	{8,-2},
	{8 ,-3.5},
	{-8.5,-3.5},
	{-8.5,-5},
	{8,-5},
	{8,-10.0},
	{6.5,-10.0},
	{6.5, -6.5},
	{5,-6.5},
	{5,-10.0},
	{3.5,-10.0},
	{3.5,-6.5},
	{2,-6.5},
	{2,-10.0},
	{0.5,-10.0},
	{0.5,-6.5},
	{-1,-6.5},
	{-1,-10.0},
	{-2.5,-10.0},
	{-2.5,-6.5},
	{-4,-6.5},
	{-4,-10.0},
	{-5.5,-10.0},
	{-5.5,-6.5},
	{-7,-6.5},
	{-7,-10.5},
	{-8.5,-6.5},
	{-8.5,-10.0},
	{-14.0,-10.0},
	{-14.0,6},
	{13.5,6},
	{13.5,-9},
	{13,-9},
	{13,4.5},
	{-13,4.5},
	{-13,-9},
	{-11.5,-9},
	{-11.5,5.0},
	{-8.5,5.0},
	{-8.5,1.5},
	{8.5,1.5},
	{8.5,2.5},
	{-7.5,2.5},
	{-7.5,3.5},
	{8,3.5},
	{11.5,4.0},
	{11.5,-9.5},
	{12.5,-9.5},
	{12.5,4.5},
	{13.5,-10.0}
};

bool moveToWaypoint(double &xLocation, double &yLocation, unsigned long elapsedTime) {
        bool output;

        if (elapsedTime < WAYPOINT_PAUSE_TIME) {
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

int intermediatePoints(double x1, double y1, double x2, double y2) {
	int output;

	output = static_cast<int>(waypointDistance(x1, y1, x2, y2) * POINTS_PER_DISTANCE);

	return output;
}

double waypointDistance(double x1, double y1, double x2, double y2) {
	double output;

	output = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
	output = sqrt(output);

	return output;
}

double waypointDistanceX(double x1, double x2) {
	double output;

	output = (x1 - x2) * (x1 - x2);
	output = sqrt(output);

	return output;
}

double waypointDistanceY(double y1, double y2) {
	return waypointDistanceX(y1, y2);
}

double waypointLength(double x1, double y1, double x2, double y2) {
	double output;

	output = waypointDistance(x1, y1, x2, y2) / static_cast<double>(intermediatePoints(x1, y1, x2, y2));

	return output;
}

double waypointLengthX(double x1, double y1, double x2, double y2) {
	double output;

	output = waypointDistanceX(x1, x2) / static_cast<double>(intermediatePoints(x1, y1, x2, y2));
	if (x2 < x1) {
		output = output * -1;
	}

	return output;
}

double waypointLengthY(double x1, double y1, double x2, double y2) {
	double output;

	output = waypointDistanceY(y1, y2) / static_cast<double>(intermediatePoints(x1, y1, x2, y2));
	if (y2 < y1) {
		output = output * -1;
	}

	return output;
}

void createTravelPath() {
	int i, j;
	int offset;
	int pointCount;
	double pointDistanceX;
	double pointDistanceY;
	double x1, x2, y1, y2;

	for (i = 0; i < DELAY_NUMBER; i++) {
		travelPath[0].push_back(newWaypoints[0][0]);
		travelPath[1].push_back(newWaypoints[0][1]);
	}
	for (i = 0; i < (NEW_WAYPOINT_NUMBER - 1); i++) {
		x1 = newWaypoints[i][0];
		y1 = newWaypoints[i][1];
		x2 = newWaypoints[i + 1][0];
		y2 = newWaypoints[i + 1][1];
		pointCount = intermediatePoints(x1, y1, x2, y2);
		pointDistanceX = waypointLengthX(x1, y1, x2, y2);
		pointDistanceY = waypointLengthY(x1, y1, x2, y2);
		for (j = 0; j < pointCount; j++) {
			travelPath[0].push_back(newWaypoints[i][0] + j * pointDistanceX);
			travelPath[1].push_back(newWaypoints[i][1] + j * pointDistanceY);
		}
	}
	travelPath[0].push_back(newWaypoints[i][0]);
	travelPath[1].push_back(newWaypoints[i][1]);
}

bool newMoveToWaypoint(double &xLocation, double &yLocation, unsigned long elapsedTime) {
        bool output;

	if (!isPathCreated) {
		createTravelPath();
		isPathCreated = true;
	}

        if (elapsedTime < WAYPOINT_DWELL_TIME) {
                output = false;
		if (isFirstPoint) {
			xLocation = (travelPath[0])[currentWaypoint];
			yLocation = (travelPath[1])[currentWaypoint];
			isFirstPoint = false;
		}
        } else {
                output = true;
		if (currentWaypoint < travelPath[0].size()) {
			++currentWaypoint;
		} else { // Reset waypoint to restart search
			currentWaypoint = 0;
		}
		xLocation = (travelPath[0])[currentWaypoint];
		yLocation = (travelPath[1])[currentWaypoint];
        }

        return output;
}

