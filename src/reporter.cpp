/*
 * File: reporter.cpp
 * Author: Stewart Nash (Aerobotics)
 * Date: June 1, 2019
 * Version: 0.0.0
 * Description: This class contains the reporting functions 
 * for the drone simulation. In effect, it reports the found 
 * April-tags. 
 */
#include "reporter.h"
#include <vector>

Reporter::Reporter() {

}

bool Reporter::isComplete() {
	if (tagList.size() >= MAXIMUM_TAGS) {
		return true;
	}

	return false;
}

bool Reporter::addTag(unsigned int input) {
	return addTag(input, 0, 0);
}

bool Reporter::addTag(unsigned int input, double coordinateX, double coordinateY) {
	bool isUnique;

	isUnique = true;
	for (int i = 0; i < tagList.size(); i++) {
		if (tagList[i] == input) {
			isUnique = false;
		}
	}

	if (isUnique) {
		tagList.push_back(input);
		xCoordinates.push_back(coordinateX);
		yCoordinates.push_back(coordinateY);
	}

	return isUnique;
}

std::vector<unsigned int> Reporter::getTags() {
	std::vector<unsigned int> output;

	output = tagList;

	return output;
}

std::vector<double> Reporter::getXCoordinates() {
	std::vector<double> output;

	output = xCoordinates;

	return output;	
}

std::vector<double> Reporter::getYCoordinates() {
	std::vector<double> output;

	output = yCoordinates;

	return output;	
}

void Reporter::resetTags() {
	tagList.clear();
	xCoordinates.clear();
	yCoordinates.clear();
}
