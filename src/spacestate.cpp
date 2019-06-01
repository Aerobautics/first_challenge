/*
 * File: spacestate.cpp
 * Author: Aerobotics (Stewart Nash)
 * Date: May 31, 2019
 * Version: 0.0.0
 * Description: This contains the implementation of the space search
 * functions.
 */
#include "spacestate.h"
#include <cmath>
#include <vector>

SpaceState worldSpace[WORLD_WIDTH][WORLD_LENGTH];
std::vector<double> forbiddenPoints[NUMBER_OF_WALLS_SS];

inline int convertLocation(double location) {
	return static_cast<int>(location / SPACE_FACTOR);
}

inline int xConvertPoint(double xLocation) {
	return static_cast<int>((xLocation - X_BUFFER) / SPACE_FACTOR);
}

inline int yConvertPoint(double yLocation) {
	return static_cast<int>((yLocation - Y_BUFFER) / SPACE_FACTOR);
}

inline double xConvertCell(int xCell) {
	return static_cast<double>(xCell) * SPACE_FACTOR + X_BUFFER;
}

inline double yConvertCell(int yCell) {
	return static_cast<double>(yCell) * SPACE_FACTOR + Y_BUFFER;
}

double distance(int x1, int y1, int x2, int y2) {
        double output;

        output = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
        output = sqrt(output);

        return output;
}

void initializeSpace(double xLocation, double yLocation) {
        int i, j;
        int x, y;
	int count;
	
        for (i = 0; i < WORLD_WIDTH; i++) {
                for (j = 0; j < WORLD_LENGTH; j++) {
                        worldSpace[i][j] = UNKNOWN;
                }
        }

        for (i = 0; i < WORLD_WIDTH; i++) {
                for (j = 0; j < WORLD_LENGTH; j++) {
                        x = xConvertPoint(xLocation);
                        y = yConvertPoint(yLocation);
                        if (distance(x, y, i, j) < 1.0001) {
                                worldSpace[i][j] = CURRENT;
                        }
                }
        }

	for (j = 0; j < NUMBER_OF_WALLS_SS; j++) {
		//std::cout << std::endl << "Wall " << j << std::endl;
		count = (forbiddenPoints[j]).size() / 2;
		for (i = 0; i < count; i++) {
			x = xConvertPoint((forbiddenPoints[j])[2 * i]);
			y = yConvertPoint((forbiddenPoints[j])[2 * i + 1]);
			//x = xConvertPoint((forbiddenPoints[j]).at(2 * i));
			//y = yConvertPoint((forbiddenPoints[j]).at(2 * i + 1));
			worldSpace[x][y] = FORBIDDEN;
			//std::cout << "(" << x << ", " << y << ") ";
			//if (i > 0 && i % 4 == 0) {
			//	std::cout << std::endl;
			//}
		} 
	}
	// Insert buffer zone
	for (j = 0; j < NUMBER_OF_WALLS_SS; j++) {
		count = (forbiddenPoints[j]).size() / 2;
		for (i = 0; i < count; i++) {
			x = xConvertPoint((forbiddenPoints[j])[2 * i]);
			y = yConvertPoint((forbiddenPoints[j])[2 * i + 1]);
			// Buffer zone
			for (int m = 0; m < WORLD_WIDTH; m++) {
				for (int n = 0; n < WORLD_LENGTH; n++) {
					if (distance(x, y, i, j) < sqrt(2.001)) {
						worldSpace[m][n] = FORBIDDEN;
					}					
				}
			}
		} 
	}
}

bool moveTo(double &xLocation, double &yLocation, unsigned long elapsedTime) {
        bool output;
        int i, j;
        int x, y;
        double temporary = 1000000.00;
	double temporary_x, temporary_y;

        x = xConvertPoint(xLocation);
        y = yConvertPoint(yLocation);
        if (elapsedTime < DWELL_TIME) {
                output = false;
        } else {
                output = true;
                for (i = 0; i < WORLD_WIDTH; i++) {
                        for (j = 0; j < WORLD_LENGTH; j++) {
                                if (distance(x, y, i, j) < temporary) {
                                        if (worldSpace[i][j] == UNKNOWN) {
						temporary_x = xConvertCell(i);
						temporary_y = yConvertCell(j);
                                                temporary = distance(x, y, i, j);
                                        }
                                }
                        }
                }
		xLocation = temporary_x;
		yLocation = temporary_y;
        }

        return output;
}


void updateSpace(double xLocation, double yLocation) {
        int i, j;
        int x, y;

        for (i = 0; i < WORLD_WIDTH; i++) {
                for (j = 0; j < WORLD_LENGTH; j++) {
                        if (worldSpace[i][j] == CURRENT) {
                                worldSpace[i][j] = SEARCHED;
                        }
                }
        }

        for (i = 0; i < WORLD_WIDTH; i++) {
                for (j = 0; j < WORLD_LENGTH; j++) {
                        x = xConvertPoint(xLocation);
                        y = yConvertPoint(yLocation);
                        if (distance(x, y, i, j) < 1.0) {
                                worldSpace[i][j] = CURRENT;
                        }
                }
        }
}
