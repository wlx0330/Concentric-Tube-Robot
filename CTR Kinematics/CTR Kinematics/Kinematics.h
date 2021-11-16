#pragma once
#include <iostream>
#include <vector>
#include <functional>
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
	void CTRIK(
		CTR& ctr,
		const blaze::StaticVector<double, 3UL>& target);

	// spaticl jacobian matrix
	blaze::StaticMatrix<double, 3UL, 6UL, blaze::columnMajor> Jacob;

	// spaticl jacobian matrix inverse
	blaze::StaticMatrix<double, 6UL, 3UL, blaze::columnMajor> Jacob_inv;

private:
	// solve spatial jacobian matrix
	void SolveJacob(
		const CTR& ctr,
		const blaze::StaticVector<double, 3UL>& target);

	// tube translation limits
	void LimitConfig(
		const CTR& ctr,
		blaze::StaticVector<double, 6UL>& config);
};