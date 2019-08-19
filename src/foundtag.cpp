/*
* File: foundtag.cpp
* Author: Stewart Nash (Aerobotics)
* Date: July 6, 2019
* Version: 0.0.0
* Description: This class is an encapsulation of the April-tags
* which may be found.
*/
#include "foundtag.h"

FoundTag::FoundTag() {
	isCoordinateValid = false;
	isLocationValid = false;
	isAreaValid = false;
	isVelocityValid = false;
	isIdentificationValid = false;
	isDetectionValid = false;
	isTimeValid = false;
}

FoundTag::FoundTag(int id) {
	isCoordinateValid = false;
	isLocationValid = false;
	isAreaValid = false;
	isVelocityValid = false;
	isIdentificationValid = false;
	isDetectionValid = false;
	isTimeValid = false;

	identification = id;
	isIdentificationValid = true;
}

FoundTag::FoundTag(int id, enum DetectionMode detection) {
	isCoordinateValid = false;
	isLocationValid = false;
	isAreaValid = false;
	isVelocityValid = false;
	isIdentificationValid = false;
	isDetectionValid = false;
	isTimeValid = false;

	identification = id;
	isIdentificationValid = true;
	detectionMode = detection;
	isDetectionValid = true;
}

FoundTag::FoundTag(int id, enum DetectionMode detection, int coordinateX, int coordinateY) {
	isCoordinateValid = false;
	isLocationValid = false;
	isAreaValid = false;
	isVelocityValid = false;
	isIdentificationValid = false;
	isDetectionValid = false;
	isTimeValid = false;

	identification = id;
	isIdentificationValid = true;
	detectionMode = detection;
	isDetectionValid = true;
	xCoordinate = coordinateX;
	yCoordinate = coordinateY;
	isCoordinateValid = true;
}

bool FoundTag::getIsCoordinateValid() {
	return isCoordinateValid;
}

bool FoundTag::getIsLocationValid() {
	return isLocationValid;
}

bool FoundTag::getIsAreaValid() {
	return isAreaValid;
}

bool FoundTag::getIsVelocityValid() {
	return isVelocityValid;
}

bool FoundTag::getIsIdentificationValid() {
	return isIdentificationValid;
}

bool FoundTag::getIsDetectionValid() {
	return isDetectionValid;
}

bool FoundTag::getIsTimeValid() {
	return isTimeValid;
}

int FoundTag::getIdentification() {
	return identification;
}

enum DetectionMode FoundTag::getDetectionMode() {
	return detectionMode;
}

double FoundTag::getXLocation() {
	return xLocation;
}

double FoundTag::getYLocation() {
	return yLocation;
}

double FoundTag::getXCoordinate() {
	return xCoordinate;
}

double FoundTag::getYCoordinate() {
	return yCoordinate;
}

double FoundTag::getaArea() {
	return area;
}

double FoundTag::getXVelocity() {
	return xVelocity;
}

double FoundTag::getYVelocity() {
	return yVelocity;
}

unsigned long FoundTag::getTime() {
	return time;
}

void FoundTag::setIdentification(int input) {
	identification = input;
	isIdentificationValid = true;
}

void FoundTag::setDetectionMode(DetectionMode input) {
	detectionMode = input;
	if (input != NOT_SPECIFIED) {
		isDetectionValid = true;
	} else {
		isDetectionValid = false;
	}
}

void FoundTag::setLocations(double locationX, double locationY) {
	xLocation = locationX;
	yLocation = locationY;
	isLocationValid = true;
} 

void FoundTag::setCoordinates(double coordinateX, double coordinateY) {
	xCoordinate = coordinateX;
	yCoordinate = coordinateY;
	isCoordinateValid = true;
}

void FoundTag::setVelocity(double velocityX, double velocityY) {
	xVelocity = velocityX;
	yVelocity = velocityY;
	isVelocityValid = true;
}

void FoundTag::setArea(double tagArea) {
	area = tagArea;
	isAreaValid = true;
}

void FoundTag::setTime(unsigned long input) {
	time = input;
	isTimeValid = true;
}