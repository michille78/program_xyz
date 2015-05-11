//------------------------------------------------------------------------------
// <copyright file="mathXyz.h" company="Noitom">
//     Copyright (c) Noitom Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------
// 2015.5.8   xyz


#pragma once

#include <math.h>
#include "kinect.h"

class kinectXyz
{
public:
	/// <summary>
	/// Constructor
	/// </summary>
	kinectXyz();

	/// <summary>
	/// Destructor
	/// </summary>
	~kinectXyz();





};



/// <summary>
/// Constructor
/// </summary>
kinectXyz::kinectXyz() 
{
	
}


/// <summary>
/// Destructor
/// </summary>
kinectXyz::~kinectXyz()
{
	
}


/// <summary>
/// Calculate the distance of two joints
/// </summary>
/// <param name="jointA">ont joint</param>
/// <param name="jointB">the other joint</param>
/// <returns>result of the distance</returns>
float JointsDistance(CameraSpacePoint jointA, CameraSpacePoint jointB)
{
	double distanceSquare;
	float distance;
	distanceSquare = pow(jointA.X - jointB.X, 2) + pow(jointA.Y - jointB.Y, 2) + pow(jointA.Z - jointB.Z, 2);
	distance = sqrt(distanceSquare);
	return distance;
}
