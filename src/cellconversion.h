/*
 * File: cellconversion.h
 * Author: Stewart Nash
 * Description: Converts wall locations to grid coordinates for
 * statespace.h. Code should be extended and made more flexible.
 */
#include <cmath>
#include <vector>

const int NUMBER_OF_WALLS  = 7;
const int X_AXIS = 0;
const int Y_AXIS = 1;
const int Z_AXIS = 2;
/*
 Wall 13
 Wall 15
 Wall 2
 Wall 20
 Wall 3
 Wall 4
 Wall 5
*/

double wallSize[][3] = {
        {20, 0.15, 2.5},
        {10, 0.15, 2.5},
        {30, 0.15, 2.5},
        {10, 0.15, 2.5},
        {18, 0.15, 2.5},
        {30, 0.15, 2.5},
        {18 ,0.15, 2.5}
};

double wallLocalLocation[][3] = {
        {0, 0, 1.25},
        {0, 0, 1.25},
        {0, 0, 1.25},
        {0, 0, 1.25},
        {0, 0, 1.25},
        {0, 0, 1.25},
        {0, 0, 1.25}
};

double wallLocalAngle[][3] = {
        {0, -0, 0},
        {0, -0, 0},
        {0, -0, 0},
        {0, -0, 0},
        {0, -0, 0},
        {0, -0, 0},
        {0, -0, 0}
};

double wallLocation[][3] = {
        {0, 0, 0},
        {-10, -2, 0},
        {0, -11, 0},
        {10, -2, 0},
        {15, -2, 0},
        {0, 7, 0},
        {-15, -2, 0},
};

double wallAngles[][3] = {
        {0, -0, 0},
        {0, 0, -1.5708},
        {0, -0, 0},
        {0, 0, -1.56904},
        {0, -0, 1.5708},
        {0, -0, 3.14159},
        {0, 0, -1.5708}
};

std::vector<double> getPoints(int wallNumber, double resolution) {
	std::vector<double> output;
	int numberOfPoints;
	double width, length;
	double i, j;
	double x, y;
	double xCenter, yCenter;
	double rotationAngle;

	if (wallNumber < NUMBER_OF_WALLS && wallNumber >= 0) {
		width = wallSize[wallNumber][X_AXIS];
		length = wallSize[wallNumber][Y_AXIS];
		xCenter = wallLocation[wallNumber][X_AXIS];
		yCenter = wallLocation[wallNumber][Y_AXIS];
		rotationAngle = wallAngles[wallNumber][Z_AXIS];
		for (i = 0; i <= width; i += resolution) {
			for (j = 0; j <= length; j += resolution) {
				x = xCenter + std::abs(cos(rotationAngle)) * (i - width / 2.0) - std::abs(sin(rotationAngle)) * (j - length / 2.0);
				y = yCenter + std::abs(cos(rotationAngle)) * (j - length / 2.0) + std::abs(sin(rotationAngle)) * (i - width / 2.0);
				output.push_back(x);
				output.push_back(y);				
			}
		}		
	} else {

	}

	return output;
}

std::vector<double> getPoints(int wallNumber) {
	std::vector<double> output;
	double resolution;

	if (wallNumber < NUMBER_OF_WALLS && wallNumber >= 0) {
		resolution = wallSize[wallNumber][Y_AXIS];
		output = getPoints(wallNumber, resolution);
	} else {

	}

	return output;
}

std::vector<int> getCells() {
	std::vector<int> output;

	return output;
}
