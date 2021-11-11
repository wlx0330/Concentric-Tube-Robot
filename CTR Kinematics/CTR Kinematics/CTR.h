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

	// solve CTR forward kinematics overload
	void SolveFK();

	// legacy, redone in kinematics class
	// solve CTR forward kinematics
	void SolveFK(
		const blaze::StaticVector<double, 3UL>& val_Tran,
		const blaze::StaticVector<double, 3UL>& val_Rot);

	// display tube translation
	blaze::StaticVector<double, 3UL> GetTran();

	// display tube rotation
	blaze::StaticVector<double, 3UL> GetRot();

	// display tube translation
	std::string GetTran(int i);

	// display tube rotation
	std::string GetRot(int i);

	// display proximal end position
	std::string GetProx();

	// display distal end position
	std::string GetDist();

	// set tube translation
	bool SetTran(int i, double val);

	// set tube rotation
	bool SetRot(int i, double val);

	// change robot configuration
	bool SetConfig(const blaze::StaticVector<double, 6UL>& config);

	// psudo matrix inverse
	blaze::DynamicMatrix<double> pinv(const blaze::DynamicMatrix<double>& M, const double& tol);

	// class properties
	BVP bvp; //bvp solver class
	BC bc; //CTR boundary condition class
	// int n; //tube count
	int m; //segment count
	static std::array<Tube, 3UL> tubes; //tube class
	std::array<blaze::DynamicMatrix<double, blaze::columnMajor>, 3UL> shapes;

	// legacy
	blaze::StaticVector<double, 3UL> TubeTran; //tube translation column vector
	blaze::StaticVector<double, 3UL> TubeRot; //tube rotataion column vector
	// combine rot and tran as input, then use functions to get rot and tran

	blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor> R_tip; //tip rotation matrix
	blaze::StaticVector<double, 3UL> DistalEnd;
	// robot input configuration
	blaze::StaticVector<double, 6UL> config; // tran (m) + rot (rad)

private:
	// determine boudnary conditions
	void SolveBC();

	// determine bvp function
	void GetFunc();

	// solve bvp function
	void SolveBVP(int n_sample);

	// solve CTR shape
	void SolveShape();

	// cumulative trapzoid integral to compute tangent vectors
	blaze::DynamicMatrix<double, blaze::columnMajor> cumtrapz(
		const blaze::DynamicVector<double, blaze::rowVector>& x_int,
		const blaze::DynamicMatrix<double>& y_int);

};