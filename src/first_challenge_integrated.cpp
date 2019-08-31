/*
 * Author: Aerobotics (Stewart Nash)
 * File: first_challenge_integrated.cpp
 * Description: Node for first drone challenge with integrated
 * computer vision and movement functions.
 */
#include "controller.h"

// Take functionality out of main for multiple methods of use.

int main(int argc, char* argv[])
{
	Controller controller;
	
	controller.testNavigation(argc, argv);

	return 0;
}
