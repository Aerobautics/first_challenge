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

/*
const int WAYPOINT_NUMBER_1 = 54;
const int WAYPOINT_NUMBER_2 = 55;
const int WAYPOINT_NUMBER_3 = 47;
const int WAYPOINT_NUMBER_4 = 34;
const int DELAY_NUMBER = 15;
const unsigned long WAYPOINT_DWELL_TIME = 15;
const unsigned long WAYPOINT_PAUSE_TIME = 250;
const double POINTS_PER_DISTANCE = 3.0;

extern bool isFirstPoint;
extern bool isPathCreated;
extern int currentWaypoint;
extern int waypointNumber;
extern std::vector<double> travelPath[2];
extern double waypoints_1[WAYPOINT_NUMBER_1][2];
extern double waypoints_2[WAYPOINT_NUMBER_2][2];
extern double waypoints_3[WAYPOINT_NUMBER_3][2];
extern double waypoints_4[WAYPOINT_NUMBER_4][2];
*/

const int WAYPOINT_NUMBER_1 = 54;
const int WAYPOINT_NUMBER_2 = 55;
const int WAYPOINT_NUMBER_3 = 47;
const int WAYPOINT_NUMBER_4 = 34;
const int DELAY_NUMBER = 15;
const unsigned long WAYPOINT_DWELL_TIME = 15;
const unsigned long WAYPOINT_PAUSE_TIME = 250;
const double POINTS_PER_DISTANCE = 3.0;


class Waypoint {
	public:
		Waypoint();
		bool oldMoveToWaypoint(double &xLocation, double &yLocation, unsigned long elapsedTime);
		bool moveToWaypoint(double &xLocation, double &yLocation, unsigned long elapsedTime);
	private:
		bool isFirstPoint;
		bool isPathCreated;
		int currentWaypoint;
		int waypointNumber;
		std::vector<double> travelPath[2];

		/*
		double waypoints_1[WAYPOINT_NUMBER_1][2];
		double waypoints_2[WAYPOINT_NUMBER_2][2];
		double waypoints_3[WAYPOINT_NUMBER_3][2];
		double waypoints_4[WAYPOINT_NUMBER_4][2];
		*/

		int intermediatePoints(double x1, double y1, double x2, double y2);
		double waypointDistance(double x1, double y1, double x2, double y2);
		double waypointDistanceX(double x1, double x2);
		double waypointDistanceY(double y1, double y2);
		double waypointLength(double x1, double y1, double x2, double y2);
		double waypointLengthX(double x1, double y1, double x2, double y2);
		double waypointLengthY(double x1, double y1, double x2, double y2);
		void createTravelPath();

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
};


