/*
 * File: waypoint.h
 * Author: Stewart Nash (Aerobotics)
 * Date: May 31, 2019
 * Version: 0.0.0
 * Description: This contains the definitions of the waypoint functions and
 * accessories for use in space search.
 */
#pragma once
#include <vector>

const int WAYPOINT_NUMBER = 54;
const int NEW_WAYPOINT_NUMBER = 55;
const int DELAY_NUMBER = 15;
const unsigned long WAYPOINT_DWELL_TIME = 15;
const unsigned long WAYPOINT_PAUSE_TIME = 250;
const double POINTS_PER_DISTANCE = 3.0;

extern bool isFirstPoint;
extern bool isPathCreated;
extern int currentWaypoint;
extern std::vector<double> travelPath[2];
extern double waypoints[WAYPOINT_NUMBER][2];
extern double newWaypoints[NEW_WAYPOINT_NUMBER][2];

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
