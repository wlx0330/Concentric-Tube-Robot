#pragma once
#include <iostream>
#include <chrono>
#include <string>
#include <blaze/Math.h>

#include "CTR.h"
#include "Kinematics.h"



class Controller
{
public:
	// constructor
	Controller();

	//menu function
	void ControllerMenu();

	//display column vectors
	std::string DispVec3(const blaze::StaticVector<double, 3UL>& pos);

	// test method
	void Test();

	// class member
	Kinematics kinematics;
	std::chrono::duration<double, std::milli> timer;
	blaze::StaticVector<double, 3UL> target;

private:
	CTR ctr;
};