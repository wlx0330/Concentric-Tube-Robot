#pragma once

#include <iostream>
#include <vector>
#include <functional>
#include <future>
#include <blaze\Math.h>
#include "BC.h"

// rename the type because it is way too long
using bvfun = std::function<blaze::StaticVector<double, 6UL>(const blaze::StaticVector<double, 6UL>& y)>;

class BVP
{
public:
	// constructor
	BVP();

	// determine Ja_bvp
	void GetJa(const double& H);

	// determine bvp residue
	blaze::StaticVector<double, 3UL> GetRes(
		const std::vector<blaze::DynamicMatrix<double, blaze::columnMajor>>& y_mesh, // this is necessary
		const BC& bc);

	// parareal algorithm
	std::vector<blaze::DynamicMatrix<double, blaze::columnMajor>> parareal(
		const BC& bc,
		const blaze::StaticVector<double, 3UL>& rot,
		const blaze::StaticVector<double, 3UL>& guess);

	// RK4 algorithm
	blaze::DynamicMatrix<double, blaze::columnMajor> RK4(
		const blaze::DynamicVector<double, blaze::rowVector>& x,
		const blaze::DynamicVector<double>& y_init,
		const bvfun& func);

	// 3d-matrix squeeze function
	blaze::DynamicMatrix<double, blaze::columnMajor> squeeze3(
		const std::vector<blaze::DynamicMatrix<double, blaze::columnMajor>>& M3D);

	// class properties
	std::vector<bvfun> v_func; //bvp functions
	std::vector<blaze::DynamicVector<double, blaze::rowVector>> x_mesh;
	std::vector<blaze::DynamicMatrix<double, blaze::columnMajor>> y_mesh;
	blaze::DynamicVector<double> res_old;
	blaze::DynamicVector<double> res_new;
	blaze::StaticVector<double, 3UL> guess_old;
	blaze::StaticVector<double, 3UL> guess_new;
	blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor> Ja;
	blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor> Ja_inv;
	int n_sample; //number of samples for each segment
	int m; //number of segments
	bool isFirstIt; //fisrt iteration flag
};