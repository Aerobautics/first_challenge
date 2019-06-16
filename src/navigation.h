/*
 * File: navigation.h
 * Author: Stewart Nash (Aerobotics)
 * Date: May 31, 2019
 * Version: 0.0.0
 * Description: This class contains the navigation functions of 
 * for the drone. This includes the pathfinding and space searching
 * functions that were previoulsy included in 'spacestate' and
 * 'searchspace'.
 */
#pragma once
#include <vector>

Class Navigation {
	public:
		Navigation(); // Constructor
		bool isIgnoringTag;
		bool isTargetingTag;
		std::vector<double> nextPoint();
		std::vector<double> identifiedTags;
		std::vector<double> reportedTags;
		
};

