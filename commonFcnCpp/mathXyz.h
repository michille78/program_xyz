//------------------------------------------------------------------------------
// <copyright file="mathXyz.h" company="Noitom">
//     Copyright (c) Noitom Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------
// 2015.5.8   xyz

#pragma once

#include <math.h>

class mathXyz
{
public:
	/// <summary>
	/// Constructor
	/// </summary>
	mathXyz();

	/// <summary>
	/// Destructor
	/// </summary>
	~mathXyz();



};


float JointDistance(float P1[3], float P2[3])
{
	double distance_s;
	float distance;
	distance_s = pow((P1[0] - P2[0]), 2) + pow((P1[1] - P2[1]), 2) + pow((P1[2] - P2[2]), 2);
	distance = sqrt(distance_s);
	return distance;
}