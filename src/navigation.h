/*
 * File: navigation.h
 * Author: Stewart Nash (Aerobotics)
 * Date: May 31, 2019
 * Version: 0.0.0
 * Description:	This class contains the navigation functions of 
 *				for the drone. This includes the pathfinding and space searching
 *				functions that were previoulsy included in 'spacestate' and
 * 'searchspace'.
 */
#pragma once
#include <vector>
#include "reporter.h"
#include "spacestate.h"
#include "waypoint.h"
#include "reporter.h"
#include "foundtag.h"

std::vector<double> navigationGetPoints(int wallNumber, double resolution);
std::vector<double> navigationGetPoints(int wallNumber);
std::vector<int> navigationGetCells();

enum NavigationMethod {WAYPOINT_NAVIGATION, SEARCH_NAVIGATION, TEST_NAVIGATION};

class Navigation {
	public:
		Navigation(); // Constructor
		Navigation(double poseX, double poseY, double offsetX, double offsetY);
		bool isIgnoringTag;
		bool isTargetingTag;
		std::vector<double> nextPoint();
		double xOffset;
		double yOffset;
		double xScale;
		double yScale;
		double xPose;
		double yPose;
		bool update(double& poseX, double& poseY, unsigned long elapsedTime);
		bool update(double& poseX, double& poseY, double xPosition, double yPosition, unsigned long elapsedTime);
		NavigationMethod navigationMethod;		

		Reporter identifiedTags;
		Reporter reportedTags;

	private:
		SpaceState spaceState;
		Waypoint waypoint;		
};

