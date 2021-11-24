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
	// Controller();

	// display tube information function
	void CTRInfo();

	// main menu function
	void ControllerMenu();

	// FK menu function
	void FKMenu();

	// IK menu function
	void IKMenu();

	// test method
	void Test();

	// class member
	Kinematics kinematics;
	std::chrono::duration<double, std::milli> timer;
	blaze::StaticVector<double, 3UL> target;

private:
	//display column vectors
	std::string DispVec3(const blaze::StaticVector<double, 3UL> &pos);

	// class member object CTR
	CTR ctr;
};