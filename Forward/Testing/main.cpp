// Forward.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "ForwardKinematics.h"

int main()
{
	LinkLengths linkLengths = { 1.0, 1.0, 1.0 };
	JointAngles angles = { 0, 1.0, 1.0 };

	EndEffectorPosition position = calculateForwardKinematics(linkLengths, angles);

	std::cout << "End-effector position is: (" << position.x << ", " << position.y << ", " << position.z << ")" << std::endl;
}

