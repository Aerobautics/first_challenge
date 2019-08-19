/*
* File: foundtag.h
* Author: Stewart Nash (Aerobotics)
* Date: July 6, 2019
* Version: 0.0.0
* Description:	This class stores data about putative tags which are found with
*				vision processing. It stores only the information necessary to track
*				them.
*/
#pragma once
#include <vector>

enum DetectionMode { NOT_SPECIFIED, APRILTAG_DETECTOR, CONTOUR_METHOD, TRACKING_METHOD};

class FoundTag {
public:
	FoundTag(); // Constructor
	FoundTag(int id);
	FoundTag(int id, enum DetectionMode detection);
	FoundTag(int id, enum DetectionMode detection, int coordinateX, int coordinateY);

	bool getIsCoordinateValid();
	bool getIsLocationValid();
	bool getIsAreaValid();
	bool getIsVelocityValid();
	bool getIsIdentificationValid();
	bool getIsDetectionValid();
	bool getIsTimeValid();

	int getIdentification();
	enum DetectionMode getDetectionMode();
	double getXLocation(); // Vehicle location
	double getYLocation(); // Vehicle location
	double getXCoordinate();
	double getYCoordinate();
	double getaArea();
	double getXVelocity();
	double getYVelocity();
	unsigned long getTime();

	void setIdentification(int input);
	void setDetectionMode(enum DetectionMode input);
	void setLocations(double locationX, double locationY);
	void setCoordinates(double coordinateX, double coordinateY);
	void setVelocity(double velocityX, double velocityY);
	void setArea(double tagArea);
	void setTime(unsigned long input);

private:
	int identification;
	enum DetectionMode detectionMode;
	double xLocation;
	double yLocation;
	double xCoordinate;
	double yCoordinate;
	double area;
	double xVelocity;
	double yVelocity;
	unsigned long time;

	bool isCoordinateValid;
	bool isLocationValid;
	bool isAreaValid;
	bool isVelocityValid;
	bool isIdentificationValid;
	bool isDetectionValid;
	bool isTimeValid;
};