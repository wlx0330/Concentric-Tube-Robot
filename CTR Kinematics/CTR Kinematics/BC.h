#pragma once

#include <iostream>
#include <set>

#include <blaze/Math.h>

class BC
{
public:

	// find unique elements from matrix and sort into vector
	blaze::DynamicVector<double> GetUnique(const blaze::StaticMatrix<double, 3UL, 3UL>& BV);

	// class properties
	blaze::DynamicVector<double> points; // boundary points
	blaze::DynamicMatrix<int, blaze::columnMajor> curvatures; //segment curvatures
	blaze::StaticMatrix<int, 3UL, 2UL, blaze::columnMajor> intervals; //segment intervals
	blaze::DynamicMatrix<int, blaze::columnMajor> types; //boundary condition types
};