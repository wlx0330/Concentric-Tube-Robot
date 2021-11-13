#pragma once
#include <iostream>
#include <blaze/Math.h>
#include "CTR.h"



class Kinematics
{
public:
	// CTR forward kinematics 
	void CTRFK(
		CTR& ctr,
		const blaze::StaticVector<double, 6UL>& config);

	// CTR inverse kinematics
	void CTRIK();


};