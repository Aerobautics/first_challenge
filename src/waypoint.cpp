/*
 * File: waypoint.cpp
 * Author: Stewart Nash (Aerobotics)
 * Date: May 31, 2019
 * Version: 0.0.0
 * Description: This contains the implementation of the waypoint functions and
 * accessories for use in space search.
 */
#include "waypoint.h"
#include <vector>
#include <cmath>

bool isFirstPoint = true;
bool isPathCreated = false;
int currentWaypoint = 0;
int waypointNumber = 2;
std::vector<double> travelPath[2];


Waypoint::Waypoint() {
	isFirstPoint = true;
	isPathCreated = false;
	currentWaypoint = 0;
	waypointNumber = 2;
}

bool Waypoint::oldMoveToWaypoint(double &xLocation, double &yLocation, unsigned long elapsedTime) {
        bool output;

        if (elapsedTime < WAYPOINT_PAUSE_TIME) {
                output = false;
		if (isFirstPoint) {
			xLocation = waypoints_1[currentWaypoint][0];
			yLocation = waypoints_1[currentWaypoint][1];
			isFirstPoint = false;
		}
        } else {
                output = true;
		if (currentWaypoint < WAYPOINT_NUMBER_1) {
			++currentWaypoint;
		} else { // Reset waypoint to restart search
			currentWaypoint = 0;
		}
		xLocation = waypoints_1[currentWaypoint][0];
		yLocation = waypoints_1[currentWaypoint][1];
        }

        return output;
}

bool Waypoint::moveToWaypoint(double &xLocation, double &yLocation, unsigned long elapsedTime) {
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

int Waypoint::intermediatePoints(double x1, double y1, double x2, double y2) {
	int output;

	output = static_cast<int>(waypointDistance(x1, y1, x2, y2) * POINTS_PER_DISTANCE);

	return output;
}

double Waypoint::waypointDistance(double x1, double y1, double x2, double y2) {
	double output;

	output = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
	output = sqrt(output);

	return output;
}

double Waypoint::waypointDistanceX(double x1, double x2) {
	double output;

	output = (x1 - x2) * (x1 - x2);
	output = sqrt(output);

	return output;
}

double Waypoint::waypointDistanceY(double y1, double y2) {
	return waypointDistanceX(y1, y2);
}

double Waypoint::waypointLength(double x1, double y1, double x2, double y2) {
	double output;

	output = waypointDistance(x1, y1, x2, y2) / static_cast<double>(intermediatePoints(x1, y1, x2, y2));

	return output;
}

double Waypoint::waypointLengthX(double x1, double y1, double x2, double y2) {
	double output;

	output = waypointDistanceX(x1, x2) / static_cast<double>(intermediatePoints(x1, y1, x2, y2));
	if (x2 < x1) {
		output = output * -1;
	}

	return output;
}

double Waypoint::waypointLengthY(double x1, double y1, double x2, double y2) {
	double output;

	output = waypointDistanceY(y1, y2) / static_cast<double>(intermediatePoints(x1, y1, x2, y2));
	if (y2 < y1) {
		output = output * -1;
	}

	return output;
}

void Waypoint::createTravelPath() {
	int i, j;
	int offset;
	int pointCount;
	int waypointCount;
	double (*waypoints)[2];
	double pointDistanceX;
	double pointDistanceY;
	double x1, x2, y1, y2;

	switch (waypointNumber) {
		case 1:
			waypoints = waypoints_1;
			waypointCount = WAYPOINT_NUMBER_1;
			break;
		case 2:
			waypoints = waypoints_2;
			waypointCount = WAYPOINT_NUMBER_2;
			break;
		case 3:
			waypoints = waypoints_3;
			waypointCount = WAYPOINT_NUMBER_3;
			break;
		case 4:
			waypoints = waypoints_4;
			waypointCount = WAYPOINT_NUMBER_4;
			break;
		default:
			waypoints = waypoints_2;
			waypointCount = WAYPOINT_NUMBER_2;
			break;
	}

	for (i = 0; i < DELAY_NUMBER; i++) {
		travelPath[0].push_back(waypoints[0][0]);
		travelPath[1].push_back(waypoints[0][1]);
	}
	for (i = 0; i < (waypointCount - 1); i++) {
		x1 = waypoints[i][0];
		y1 = waypoints[i][1];
		x2 = waypoints[i + 1][0];
		y2 = waypoints[i + 1][1];
		pointCount = intermediatePoints(x1, y1, x2, y2);
		pointDistanceX = waypointLengthX(x1, y1, x2, y2);
		pointDistanceY = waypointLengthY(x1, y1, x2, y2);
		for (j = 0; j < pointCount; j++) {
			travelPath[0].push_back(waypoints[i][0] + j * pointDistanceX);
			travelPath[1].push_back(waypoints[i][1] + j * pointDistanceY);
		}
	}
	travelPath[0].push_back(waypoints[i][0]);
	travelPath[1].push_back(waypoints[i][1]);
}



double waypoints_1[WAYPOINT_NUMBER_1][2] = { // Size of 54x2
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

double waypoints_2[WAYPOINT_NUMBER_2][2] = { // Size of 54x2
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

//Stay 1 meter from wall with 1 meter interval
double waypoints_3[WAYPOINT_NUMBER_3][2] = {
	{10,-10},
	{-8,-10},
	{-8,-9},
	{9,-9},
	{9,-8},
	{-8,-8},
	{-8,-7},
	{9,-7},
	{9,-6},
	{-8,-6},
	{-8,-5},
	{9,-5},
	{9, -4},
	{-8,-4},
	{-8,-3},
	{9,-3},
	{-8,-3},
	{-8, -2},
	{9,-2},
	{9,-1},
	{-9,-1},
	{-9,-10},
	{-11.1,-10},
	{-11.1, 5},
	{-12.3, 5},
	{-12.3, -10},
	{-13.6,-10},
	{-13.6,6},
	{8,6},
	{8,5},
	{-9,5},
	{-9,4},
	{8,4},
	{8,2.8},
	{-9, 2.8},
	{-9,1.5},
	{9,1.5},
	{9,6},
	{11,6},
	{11, -10},
	{12,-10},
	{12,6},
	{13,6},
	{13,-10},
	{14,-10},
	{14,6}
};

// Stay 1.5 meters from wall with 1.5 meter interval
double waypoints_4[WAYPOINT_NUMBER_4][2] = {
	{10,-9.5},
	{-8.5,-9.5},
	{-8.5,-8},
	{8.5,-8},
	{8.5,-6.5},
	{-8.5,-6.5},
	{-8.5,-5},
	{8.5, -5},
	{8.5,-3.5},
	{-8.5,-3.5,},
	{-8.5,-1.5},
	{8.5,-1.5},
	{-10,-9.5},
	{-11.5,-9.5},
	{-11.5,4},
	{-13,4},
	{-13,-9.5},
	{-13.5,-9.5},
	{-13.5,5.5},
	{8.5,5.5},
	{8.5,4},
	{-8.5,4},
	{-8.5,3},
	{8.5,3},
	{8.5,1.5},
	{-8.5,1.5},
	{9,5.5},
	{11.5,5.5},
	{11.5, -9.5},
	{12.5, -9.5},
	{12.5,5.5},
	{13.5, 5.5},
	{13.5,-9.5}
};

