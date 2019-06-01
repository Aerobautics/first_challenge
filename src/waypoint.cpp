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
std::vector<double> travelPath[2];

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

