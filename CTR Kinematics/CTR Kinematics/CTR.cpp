#include "CTR.h"

// #include <chrono>

//initialize for 3 tubes
std::array<Tube, 3UL> CTR::tubes = { Tube(), Tube(), Tube() };

//constructor
CTR::CTR()
{
	//set tube parameters ID OD Ls Lc R
	// CTR::tubes[0].SetDimension(0.8e-3, 0.92e-3, 0.199, 0.059, 1.0/25.555);
	// CTR::tubes[1].SetDimension(0.97e-3, 1.1e-3, 0.169, 0.089, 1.0/10.555);
	// CTR::tubes[2].SetDimension(1.2e-3, 1.4e-3, 0.1, 0.0, INFINITY);
	CTR::tubes[0].SetDimension(0.8e-3, 0.92e-3, 0.19, 0.06, 0.04);
	CTR::tubes[1].SetDimension(0.97e-3, 1.1e-3, 0.12, 0.08, 0.1);
	CTR::tubes[2].SetDimension(1.2e-3, 1.4e-3, 0.1, 0.0, INFINITY);

	// init input (Tran & Rot)
	// blaze::subvector(this->config, 0UL, 3UL) = { 0.199 - 0.115, 0.169 - 0.1, 0.1 - 0.08 };
	// blaze::subvector(this->config, 3UL, 3UL) = { 0.0, 0.0, 0.0 };
	blaze::subvector(this->config, 0UL, 3UL) = { 0.05, 0.0, 0.0 };
	blaze::subvector(this->config, 3UL, 3UL) = { 0.0, 0.0, 0.0 };
	//auto t1 = std::chrono::high_resolution_clock::now();
	this->SolveBC(blaze::subvector(this->config, 0UL, 3UL));
	this->SolveFunc();
	this->SolveBVP(20);
	this->SolveShape();
	//auto t2 = std::chrono::high_resolution_clock::now();
	//std::chrono::duration<double, std::milli> timer = t2 - t1;
	//std::cout << timer.count() << std::endl;
	//system("pause");
}

// change robot configuration
bool CTR::SetConfig(const blaze::StaticVector<double, 6UL>& config_new)
{
	// TODO check if input is valid 
	blaze::StaticVector<double, 3UL> TubeTran, TubeRot;
	TubeTran = blaze::subvector(config_new, 0UL, 3UL);
	TubeRot = blaze::subvector(config_new, 3UL, 3UL);

	//check if tube tran is the same
	if (!blaze::isZero(TubeTran - this->GetTran())) {
		// full FK if TubeTran changes
		this->SolveBC(TubeTran);
		this->SolveFunc();
		this->config = config_new;
		this->SolveBVP(20);
		this->SolveShape();
	}
	else if (!blaze::isZero(TubeRot - this->GetRot())) {
		// partial FK if only TubeRot changes
		this->config = config_new;
		this->SolveBVP(20);
		this->SolveShape();
	}
	return true;
}

// solve CTR shape
void CTR::SolveShape()
{
	// init variables
	blaze::StaticVector<double, 3UL> K_z, u_abs;
	blaze::DynamicMatrix<double, blaze::rowMajor> y;
	blaze::DynamicMatrix<double> u(3UL, this->bvp.n_sample); //curvature container
	blaze::DynamicMatrix<double> u_int(3UL, this->bc.m * this->bvp.n_sample, 0.0); //curvature integral container
	blaze::DynamicVector<double, blaze::rowVector> x_int(this->bc.m * this->bvp.n_sample); //function x valus for integral

	for (int i = 0; i < this->bc.m; ++i) {
		for (int j = 0; j < this->tubes.size(); ++j) {
			K_z[j] = this->tubes[j].GetKz();
			u_abs[j] = this->tubes[j].GetUabs();
			if (this->bc.curvatures(i, j) == -1) {
				u_abs[j] = 0;
			}
			else if (this->bc.curvatures(i, j) == 0) {
				u_abs[j] = 0;
				K_z[j] = 0;
			}
		}
		y = this->bvp.y_mesh[i];
		blaze::row(u, 0UL) = (K_z[0] * u_abs[0] * blaze::cos(blaze::row(y, 0UL))
			+ K_z[1] * u_abs[1] * blaze::cos(blaze::row(y, 1UL))
			+ K_z[2] * u_abs[2] * blaze::cos(blaze::row(y, 2UL))) / blaze::sum(K_z);
		blaze::row(u, 1UL) = (K_z[0] * u_abs[0] * blaze::sin(blaze::row(y, 0UL))
			+ K_z[1] * u_abs[1] * blaze::sin(blaze::row(y, 1UL))
			+ K_z[2] * u_abs[2] * blaze::sin(blaze::row(y, 2UL))) / blaze::sum(K_z);
		blaze::row(u, 2UL) = 0.0; // force u_z to be zero
		blaze::submatrix(u_int, 0UL, i * this->bvp.n_sample, 3UL, this->bvp.n_sample) = u;
		blaze::subvector(x_int, i * this->bvp.n_sample, this->bvp.n_sample) = this->bvp.x_mesh[i];
	}

	// numerical integral
	auto pos = this->cumtrapz(x_int, u_int);
	blaze::row(pos, 2UL) += this->bc.points[0UL];

	// save shape for each tube
	for (int i = 0; i < 3; ++i) {
		this->shapes[i] = blaze::submatrix(pos, 0UL, this->bc.intervals(i, 0UL) * this->bvp.n_sample,
			3UL, (this->bc.intervals(i, 1UL) - this->bc.intervals(i, 0UL) + 1) * this->bvp.n_sample);
		this->shapes[i] = blaze::round(this->shapes[i] * 1e12) * 1e-12;
	}
}

// cumulative trapzoid integral to compute tangent vectors
blaze::DynamicMatrix<double, blaze::columnMajor> CTR::cumtrapz(
	const blaze::DynamicVector<double, blaze::rowVector>& x_int,
	const blaze::DynamicMatrix<double>& y_int)
{
	blaze::DynamicMatrix<double, blaze::columnMajor> area(3UL, x_int.size() - 1);
	blaze::DynamicVector<double, blaze::rowVector> height
		= blaze::subvector(x_int, 1UL, x_int.size() - 1)
		- blaze::subvector(x_int, 0UL, x_int.size() - 1);
	blaze::DynamicMatrix<double, blaze::rowMajor> base
		= blaze::submatrix(y_int, 0UL, 0UL, 3UL, y_int.columns() - 1)
		+ blaze::submatrix(y_int, 0UL, 1UL, 3UL, y_int.columns() - 1);
	// calculate area
	for (int i = 0; i < 3; ++i) {
		// may be slow by calling row
		blaze::row(area, i) = blaze::row(base, i) * height * 0.5;
	}
	// initialize loop
	blaze::StaticVector<double, 3UL> cum_area;
	blaze::StaticMatrix<double, 3UL, 3UL> u_hat = { {0.0, -cum_area[2], cum_area[1]},
			{cum_area[2], 0.0, -cum_area[0]}, {-cum_area[1], cum_area[0], 0.0} };
	blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor> R = blaze::matexp(u_hat);
	blaze::DynamicMatrix<double, blaze::columnMajor> p_dot(3UL, x_int.size(), 0.0);
	blaze::column(p_dot, 0UL) = blaze::column(R, 2UL);

	for (int i = 0; i < x_int.size() - 1;) {
		cum_area += blaze::column(area, i++);
		u_hat = { {0.0, -cum_area[2], cum_area[1]},
			{cum_area[2], 0.0, -cum_area[0]},
			{-cum_area[1], cum_area[0], 0.0} };
		R = blaze::matexp(u_hat);
		blaze::column(p_dot, i) = blaze::column(R, 2UL);
	}
	this->R_tip = R; // save the tip orientation

	//integrate again to get postition vector
	base = blaze::submatrix(p_dot, 0UL, 0UL, 3UL, y_int.columns() - 1)
		+ blaze::submatrix(p_dot, 0UL, 1UL, 3UL, y_int.columns() - 1);
	for (int i = 0; i < 3; ++i) {
		//check speed
		blaze::row(area, i) = blaze::row(base, i) * height * 0.5;
	}
	blaze::DynamicMatrix<double, blaze::columnMajor> p(3UL, x_int.size(), 0.0);
	cum_area = 0.0;
	for (int i = 0; i < x_int.size() - 1;) {
		cum_area += blaze::column(area, i++);
		blaze::column(p, i) = cum_area;
	}
	return p;
}

// determine boudnary conditions
void CTR::SolveBC(const blaze::StaticVector<double, 3UL>& TubeTran)
{
	blaze::StaticMatrix<double, 3UL, 3UL> BV; //calculate boundary value matrix
	BV = { {-CTR::tubes[0].GetLS(), -CTR::tubes[1].GetLS(), -CTR::tubes[2].GetLS()},
		{0, 0, 0}, {CTR::tubes[0].GetLC(), CTR::tubes[1].GetLC(), CTR::tubes[2].GetLC()} };
	BV += blaze::expand<3>(blaze::trans(TubeTran));
	this->bc.points = this->bc.GetUnique(BV); //boundary value points
	this->bc.m = (int)this->bc.points.size() - 1; //number of CTR segments

	// determine the curvature at each section
	this->bc.curvatures.resize(this->bc.m, 3UL, false); //resize variable
	this->bc.curvatures = 0;
	for (int i = 0; i < this->bc.m; ++i) {
		for (int j = 0; j < 3; ++j) {
			if (BV(1, j) <= this->bc.points[i] && this->bc.points[i] < BV(2, j)) {
				this->bc.curvatures(i, j) = 1; //indicating curved section
			}
			else if (BV(0, j) <= this->bc.points[i] && this->bc.points[i] < BV(1, j)) {
				this->bc.curvatures(i, j) = -1; //indicating straight section
			}
		}
	}

	//determine the tube start and end interval
	blaze::StaticVector<int, 2UL, blaze::rowVector> interval = { 0, this->bc.m - 1 }; //initial interval
	this->bc.intervals = blaze::expand<3>(interval);
	for (int i = 1; i < this->bc.m; ++i) {
		for (int j = 0; j < 3; ++j) {
			if (this->bc.curvatures(i - 1, j) == 0 && this->bc.curvatures(i, j) != 0) {
				this->bc.intervals(j, 0) = i; //update left interval
			}
			else if (this->bc.curvatures(i - 1, j) != 0 && this->bc.curvatures(i, j) == 0) {
				this->bc.intervals(j, 1) = i - 1; //update right interval
			}
		}
	}

	//determine boundary condition type
	blaze::DynamicMatrix<int> temp = blaze::trans(blaze::abs(this->bc.curvatures));
	for (int i = 0; i < 3; ++i) {
		temp(i, this->bc.intervals(i, 0)) = 0;
	}
	this->bc.types = blaze::repeat<2UL, 1UL>(temp);
}

// determine bvp function
void CTR::SolveFunc()
{
	// initialize variables
	blaze::StaticVector<double, 3UL> K_z;
	// coefficient matrix for each segment
	blaze::StaticMatrix<double, 3UL, 3UL> u_abs, C;

	// initiate lambda function
	auto skewM = [&K_z, &u_abs]() {
		blaze::StaticMatrix<double, 3UL, 3UL> u_hat;
		u_hat = { {0.0, -K_z[2] * u_abs(2,2), K_z[1] * u_abs(1,1)},
			{K_z[2] * u_abs(2,2), 0.0, -K_z[0] * u_abs(0,0)},
			{-K_z[1] * u_abs(1,1), K_z[0] * u_abs(0,0), 0.0} };
		return u_hat;
	};
	// clear the previous container
	this->bvp.v_func.clear();

	for (int i = 0; i < this->bc.m; ++i) {
		for (int j = 0; j < 3; ++j) {
			// reset Kz and Uabs for every segment
			K_z[j] = CTR::tubes[j].GetKz();
			u_abs(j, j) = CTR::tubes[j].GetUabs();
			if (this->bc.curvatures(i, j) == -1) {
				u_abs(j, j) = 0.0;
			}
			else if (this->bc.curvatures(i, j) == 0) {
				K_z[j] = 0.0;
				u_abs(j, j) = 0.0;
			}
		}
		if (blaze::sum(K_z) == 0.0) {
			C = 0.0;
		}
		else {
			C = (Tube::GetV() + 1.0) * u_abs * skewM() / blaze::sum(K_z);
		}
		auto func = [C](const blaze::StaticVector<double, 6UL>& y)->blaze::DynamicVector<double> {
			blaze::IdentityMatrix<double> I(6UL); // assign identity matrix
			blaze::StaticMatrix<double, 6UL, 6UL> coef(I); // coefficient matrix for bvp
			blaze::submatrix(coef, 3UL, 3UL, 3UL, 3UL) = C;
			blaze::StaticVector<double, 6UL> bvpfun // bvp function vector
				= { y[3], y[4], y[5], sin(y[1] - y[2]), sin(y[2] - y[0]),sin(y[0] - y[1]) };
			return coef * bvpfun;
		};
		this->bvp.v_func.push_back(func);
	}
}

// get tube translation
blaze::StaticVector<double, 3UL> CTR::GetTran() const
{
	return blaze::subvector(this->config, 0UL, 3UL);
}

// get tube rotation
blaze::StaticVector<double, 3UL> CTR::GetRot() const
{
	return blaze::subvector(this->config, 3UL, 3UL);
}

// get CTR proximal end position
blaze::StaticVector<double, 3UL> CTR::GetProx() const
{
	return blaze::column(this->shapes[0UL], 0UL);
}

// get CTR distal end position
blaze::StaticVector<double, 3UL> CTR::GetDist() const
{
	return blaze::column(this->shapes[0], this->shapes[0UL].columns() - 1);
}

// get CTR configruation tran & rot
blaze::StaticVector<double, 6UL> CTR::GetConfig() const
{
	return this->config;
}

// solve bvp function
void CTR::SolveBVP(const int& n_sample)
{
	// update bvp object
	this->bvp.n_sample = n_sample;
	// this->bvp.m = this->m;

	// assign x-mesh
	this->bvp.x_mesh.clear();
	for (int i = 0; i < this->bc.m; ++i) {
		this->bvp.x_mesh.push_back(blaze::linspace<blaze::rowVector>
			(n_sample, this->bc.points[i], this->bc.points[i + 1]));
	}

	// initialize bvp
	int it = 0;
	this->bvp.y_mesh = this->bvp.parareal(this->bc, this->GetRot(), this->bvp.guess_new);
	this->bvp.res_new = this->bvp.GetRes(this->bvp.y_mesh, this->bc);

	// initilize Ja
	double H = 1e-3; //jacobian small step
	if (this->bvp.isFirstIt) {
		blaze::StaticVector<double, 3UL> res_diff;
		for (int i = 0; i < 3; ++i) {
			auto guess_diff = this->bvp.guess_new;
			guess_diff[i] = this->bvp.guess_new[i] + H;
			auto y_mesh_diff = this->bvp.parareal(this->bc, this->GetRot(), guess_diff);
			res_diff = this->bvp.GetRes(y_mesh_diff, this->bc);
			blaze::column(this->bvp.Ja, i) = (res_diff - this->bvp.res_new) / H;
		}
		this->bvp.Ja_inv = this->pinv(this->bvp.Ja, 1e-9); // pinv with tol of 1e-9
		this->bvp.isFirstIt = false; // remove flag
	}

	// loop until error is small
	while (blaze::l1Norm(this->bvp.res_new) > 1e-6) {
		// snapping config
		if (it > 10) {
			this->bvp.guess_new *= -0.1;
			blaze::DynamicVector<double> res_diff;
			for (int i = 0; i < 3; ++i) {
				auto guess_diff = this->bvp.guess_new;
				guess_diff[i] = this->bvp.guess_new[i] + H * 10.0;
				auto y_mesh_diff = this->bvp.parareal(this->bc, this->GetRot(), guess_diff);
				res_diff = this->bvp.GetRes(y_mesh_diff, this->bc);
				blaze::column(this->bvp.Ja, i) = (res_diff - this->bvp.res_new) / H * 0.1;
			}
			this->bvp.Ja_inv = this->pinv(this->bvp.Ja, 1e-9); // pinv with tol of 1e-9
			it = 0; // reset counter
		}
		// normal config update Ja
		else if (it > 0) {
			auto delta_F = this->bvp.res_new - this->bvp.res_old;
			blaze::StaticVector<double, 3UL> delta_x = this->bvp.guess_new - this->bvp.guess_old;
			if (delta_x.nonZeros() > 0) {
				this->bvp.Ja = this->bvp.Ja + (delta_F - this->bvp.Ja * delta_x)
					* blaze::trans(delta_x) / blaze::sqrNorm(delta_x);
			}
			this->bvp.Ja_inv = this->pinv(this->bvp.Ja, 1e-9); // pinv with tol of 1e-9
		}
		// update guess
		this->bvp.res_old = this->bvp.res_new;
		this->bvp.guess_old = this->bvp.guess_new;
		this->bvp.guess_new = this->bvp.guess_new - this->bvp.Ja_inv * this->bvp.res_old;
		this->bvp.y_mesh = this->bvp.parareal(this->bc, this->GetRot(), this->bvp.guess_new);
		this->bvp.res_new = this->bvp.GetRes(this->bvp.y_mesh, this->bc);
		it++;
	}
}

// psudo matrix inverse
blaze::DynamicMatrix<double> CTR::pinv(
	const blaze::DynamicMatrix<double>& M,
	const double& tol) const
{
	blaze::DynamicMatrix<double> U;  // The matrix for the left singular vectors
	blaze::DynamicVector<double> s;  // The vector for the singular values
	blaze::DynamicMatrix<double> V;  // The matrix for the right singular vectors
	blaze::svd(M, U, s, V);
	int n = s.size();
	blaze::DynamicMatrix<double> Sinv(n, n, 0.0);  // The diagonal matrix of inverted sigular values
	for (int i = 0; i < n; ++i) {
		blaze::abs(s[i]) < tol ? Sinv(i, i) = 0 : Sinv(i, i) = 1.0 / s[i];
	}
	return blaze::trans(U * Sinv * V);
}