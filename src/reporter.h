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

class Reporter {
	public:
		Reporter(); // Constructor
		bool isComplete();
		bool addTag(unsigned int input);
		bool addTag(unsigned int input, double coordinateX, double coordinateY);
		std::vector<unsigned int> getTags();
		std::vector<double> getXCoordinates();
		std::vector<double> getYCoordinates();
		void resetTags();
		const int MAXIMUM_TAGS = 6;

	private:
		std::vector<unsigned int> tagList;
		std::vector<double> xCoordinates;
		std::vector<double> yCoordinates;
};

