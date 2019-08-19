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
#include "foundtag.h"
#include <vector>

Reporter::Reporter() {

}

bool Reporter::isComplete() {
	if (tagList.size() >= MAXIMUM_TAGS) {
		return true;
	}

	return false;
}

bool Reporter::addTag(FoundTag input) {
	bool isUnique;

	isUnique = true;
	for (int i = 0; i < tagList.size(); i++) {
		if (tagList[i].getIdentification() == input.getIdentification()) {
			isUnique = false;
		}
	}

	if (isUnique) {
		tagList.push_back(input);
	}

	return isUnique;
}

bool Reporter::addTag(unsigned int input) {
	bool isUnique;
	FoundTag temporary(input);

	isUnique = true;
	for (int i = 0; i < tagList.size(); i++) {
		if (tagList[i].getIdentification() == input) {
			isUnique = false;
		}
	}

	if (isUnique) {
		tagList.push_back(input);
	}

	return isUnique;
}

bool Reporter::addTag(unsigned int input, double coordinateX, double coordinateY) {
	bool isUnique;
	FoundTag temporary(input);

	isUnique = true;
	for (int i = 0; i < tagList.size(); i++) {
		if (tagList[i].getIdentification() == input) {
			isUnique = false;
		}
	}

	if (isUnique) {
		temporary.setCoordinates(coordinateX, coordinateY);
		tagList.push_back(temporary);
	}

	return isUnique;
}

bool Reporter::addTag(unsigned int input, double coordinateX, double coordinateY, double tagArea) {
	bool isUnique;
	FoundTag temporary(input);

	isUnique = true;
	for (int i = 0; i < tagList.size(); i++) {
		if (tagList[i].getIdentification() == input) {
			isUnique = false;
		}
	}

	if (isUnique) {
		temporary.setCoordinates(coordinateX, coordinateY);
		temporary.setArea(tagArea);
		tagList.push_back(temporary);
	}

	return isUnique;
}

std::vector<FoundTag> Reporter::getTags() {
	std::vector<FoundTag> output;

	output = tagList;

	return output;
}

std::vector<FoundTag> Reporter::getTagList() {
	std::vector<FoundTag> output;

	output = tagList;

	return output;
}

void Reporter::resetTags() {
	tagList.clear();
}
