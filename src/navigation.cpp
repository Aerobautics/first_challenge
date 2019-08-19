/*
 * File: navigation.cpp
 * Author: Stewart Nash (Aerobotics)
 * Date: May 31, 2019
 * Version: 0.0.0
 * Description:	This class contains the navigation functions of 
 *				for the drone. This includes the pathfinding and space searching
 *				functions that were previoulsy included in 'spacestate' and
 *				'searchspace'.
 */
#include "navigation.h"
#include "cellconversion.h"
#include <iostream>

std::vector<double> navigationGetPoints(int wallNumber, double resolution) {
	return getPoints(wallNumber, resolution);
}

std::vector<double> navigationGetPoints(int wallNumber) {
	return getPoints(wallNumber);
}

std::vector<int> navigationGetCells() {
	return getCells();
}

Navigation::Navigation() {
	xOffset = 10;
	yOffset = -9;
	xScale = 1.00;
	yScale = 1.00;
	xPose = 0;
	yPose = 0;
	// Initialize spacestate values
	for (int i = 0; i < NUMBER_OF_WALLS; i++) {
		forbiddenPoints[i] = getPoints(i);
	}
	spaceState.initializeSpace(xPose + xOffset, yPose + yOffset);
	navigationMethod = WAYPOINT_NAVIGATION;	
}

Navigation::Navigation(double poseX, double poseY, double offsetX, double offsetY) {
	xOffset = offsetX;
	yOffset = offsetY;
	xScale = 1.00;
	yScale = 1.00;
	xPose = poseX;
	yPose = poseY;
	// Initialize spacestate values
	for (int i = 0; i < NUMBER_OF_WALLS; i++) {
		forbiddenPoints[i] = getPoints(i);
	}
	spaceState.initializeSpace(xPose + xOffset, yPose + yOffset);	
}

// Return point that drone should move to given elapsed time and current position
// @return value indicates whther the drone should move or not
bool Navigation::update(double& poseX, double& poseY, unsigned long elapsedTime) {
	bool output;
	double xPosition, yPosition;

	xPosition = poseX;
	yPosition = poseY;

	return update(poseX, poseY, xPosition, yPosition, elapsedTime);
}

// Reference returns point that drone should move to given elapsed time and current position
// @return value indicates whether the drone should move or not
bool Navigation::update(double& poseX, double& poseY, double xPosition, double yPosition, unsigned long elapsedTime) {
	bool output;

	poseX = xPosition / xScale + xOffset;
	poseY = yPosition / yScale + yOffset;		
	spaceState.updateSpace(poseX, poseY);
	if (navigationMethod == WAYPOINT_NAVIGATION) {
		if (waypoint.moveToWaypoint(poseX, poseY, elapsedTime)) {
			//xPosition = poseX - xOffset;
			//yPosition = poseY - yOffset;
			xPosition = xScale * (poseX - xOffset);
			yPosition = yScale * (poseY - yOffset);			
			std::cout << "moveTo: (" << xPosition / xScale + xOffset;
			std::cout << ", " << yPosition / yScale + yOffset << ")" << std::endl;
			poseX = xPosition;
			poseY = yPosition;
			output = true;
		} else { // output = waypoint.moveToWaypoint(poseX, poseY, elapsedTime);
			output = false;
		}
	} else if (navigationMethod == SEARCH_NAVIGATION) {
		if (spaceState.moveTo(poseX, poseY, elapsedTime)) {
			xPosition = poseX - xOffset;
			yPosition = poseY - yOffset;
			std::cout << "moveTo: (" << xPosition + xOffset;
			std::cout << ", " << yPosition + yOffset << ")" << std::endl;
			output = true;
		} else { // output = spaceState.moveTo(poseX, poseY, elapsedTime);
			output = false;
		}
	} else {
		if (waypoint.moveToWaypoint(poseX, poseY, elapsedTime)) {
			//xPosition = poseX - xOffset;
			//yPosition = poseY - yOffset;
			xPosition = xScale * (poseX - xOffset);
			yPosition = yScale * (poseY - yOffset);			
			std::cout << "moveTo: (" << xPosition / xScale + xOffset;
			std::cout << ", " << yPosition / yScale + yOffset << ")" << std::endl;
			poseX = xPosition;
			poseY = yPosition;
			output = true;
		} else { // output = waypoint.moveToWaypoint(poseX, poseY, elapsedTime);
			output = false;
		}
	}	

	spaceState.displaySpace();
	return output;
}
