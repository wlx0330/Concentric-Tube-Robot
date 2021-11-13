#pragma once

#include <iostream>
#include <string>
#include <array>
#include <vector>
#include <functional>
#include <blaze/Math.h>

#include "Tube.h"
#include "BC.h"
#include "BVP.h"

class CTR
{
public:
	// constructor
	CTR();

	// copy constructor
	// CTR(const CTR& ctr);

	// get tube translation
	blaze::StaticVector<double, 3UL> GetTran();

	// get tube rotation
	blaze::StaticVector<double, 3UL> GetRot();

	// set tube translation
	// bool SetTran(int i, double val);

	// set tube rotation
	// bool SetRot(int i, double val);

	// change robot configuration
	bool SetConfig(const blaze::StaticVector<double, 6UL>& config);

	// psudo matrix inverse
	blaze::DynamicMatrix<double> pinv(const blaze::DynamicMatrix<double>& M, const double& tol);

	// class properties
	BVP bvp; //bvp solver class
	BC bc; //CTR boundary condition class
	static std::array<Tube, 3UL> tubes; //tube class
	std::array<blaze::DynamicMatrix<double, blaze::columnMajor>, 3UL> shapes;
	blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor> R_tip; //tip rotation matrix
	blaze::StaticVector<double, 3UL> DistalEnd;

private:
	// robot configruation
	blaze::StaticVector<double, 6UL> config; // tran (m) + rot (rad)

	// determine boudnary conditions
	void SolveBC(const blaze::StaticVector<double, 3UL>& TubeTran);

	// determine bvp function
	void SolveFunc();

	// solve bvp function
	void SolveBVP(const int& n_sample);

	// solve CTR shape
	void SolveShape();

	// cumulative trapzoid integral to compute tangent vectors
	blaze::DynamicMatrix<double, blaze::columnMajor> cumtrapz(
		const blaze::DynamicVector<double, blaze::rowVector>& x_int,
		const blaze::DynamicMatrix<double>& y_int);
};