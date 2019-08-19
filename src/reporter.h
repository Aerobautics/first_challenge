/*
 * File: reporter.h
 * Author: Stewart Nash (Aerobotics)
 * Date: June 1, 2019
 * Version: 0.0.0
 * Description: This class contains the reporting functions 
 * for the drone simulation. In effect, it reports the found 
 * April-tags. 
 */
#pragma once
#include <vector>
#include "foundtag.h"

class Reporter {
	public:
		Reporter(); // Constructor
		bool isComplete();
		bool addTag(unsigned int input);
		bool addTag(unsigned int input, double coordinateX, double coordinateY);
		bool addTag(unsigned int input, double coordinateX, double coordinateY, double tagArea);
		bool addTag(FoundTag input);
		//bool addFoundTag(FoundTag input);
		std::vector<FoundTag> getTags();
		std::vector<FoundTag> getTagList();
		void resetTags();
		const int MAXIMUM_TAGS = 6;

	private:
		std::vector<FoundTag> foundTags;
		std::vector<FoundTag> tagList;
};

