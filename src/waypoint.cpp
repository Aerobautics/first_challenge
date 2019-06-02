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
#include <iostream>

/*
bool isFirstPoint = true;
bool isPathCreated = false;
int currentWaypoint = 0;
int waypointNumber = 2;
std::vector<double> travelPath[2];
*/

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

