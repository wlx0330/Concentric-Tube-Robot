#pragma once
#include<iostream>
#include<string>
#include <chrono>

#include"CTR.h"

class RobotController
{
public:
	//constructor
	RobotController();

	//menu function
	void ControllerMenu();

	// robot FK with timer
	void RobotFK();

	// robot IK
	CTR
		RobotIK(
			const CTR& ctr,
			const blaze::StaticVector<double, 3UL>& target);

	// calculate tip spatial jacobian
	blaze::StaticMatrix<double, 3UL, 6UL, blaze::columnMajor>
		ComputeJa(
			const CTR& ctr,
			const blaze::StaticVector<double, 3UL>& target);

	// tube translation limits
	void GetLimits(
		const CTR& ctr,
		blaze::StaticVector<double, 6UL>& config);


	// class members
	CTR ctr;
	std::chrono::duration<double, std::milli> timer;
	blaze::StaticVector<double, 3UL> target;
};