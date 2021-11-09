#include "CTR.h"

//initialize for 3 tubes
std::array<Tube, 3UL> CTR::tubes = { Tube(), Tube(), Tube() };

//constructor
CTR::CTR()
{
	//get tube number
	this->n = (int)CTR::tubes.size();

	//set tube parameters ID OD Ls Lc R
	CTR::tubes[0].SetDimension(0.8e-3, 0.92e-3, 0.19, 0.06, 0.04);
	CTR::tubes[1].SetDimension(0.97e-3, 1.1e-3, 0.12, 0.08, 0.1);
	CTR::tubes[2].SetDimension(1.2e-3, 1.4e-3, 0.1, 0.0, INFINITY);

	this->TubeTran = { 0.05, 0.0, 0.0 }; //initial translation
	this->TubeRot = { 1.570796326794897 , 0.0, 0.0 }; //initial rotation in rad
	//this->TubeRot = { 0.0 , 0.0, 0.0 }; //initial rotation in rad
}

// solve CTR forward kinematics overload
void CTR::SolveFK()
{
	this->SolveFK(this->TubeTran, this->TubeRot);
}

// solve CTR forward kinematics
void CTR::SolveFK(
	const blaze::StaticVector<double, 3UL>& val_Tran,
	const blaze::StaticVector<double, 3UL>& val_Rot)
{
	//round input to avoid numerical instability
	this->TubeTran = blaze::round(val_Tran * 1e12) * 1e-12;
	this->TubeRot = blaze::round(val_Rot * 1e12) * 1e-12;
	// this->bc = BC();
	this->SolveBC();
	this->GetFunc();
	this->SolveBVP(20);
	this->SolveShape();
	this->DistalEnd = blaze::column(this->shapes[0], this->shapes[0].columns() - 1);
	blaze::subvector(this->config, 0UL, 3UL) = this->TubeTran;
	blaze::subvector(this->config, 3UL, 3UL) = this->TubeRot;
}

// solve CTR shape
void CTR::SolveShape()
{
	blaze::StaticVector<double, 3UL> K_z;
	blaze::StaticVector<double, 3UL> u_abs;
	blaze::DynamicMatrix<double, blaze::rowMajor> y;
	blaze::DynamicMatrix<double> u(3UL, this->bvp.n_sample); //curvature container
	blaze::DynamicMatrix<double> u_int(3UL, this->m * this->bvp.n_sample, 0.0); //curvature integral container
	blaze::DynamicVector<double, blaze::rowVector> x_int(this->m * this->bvp.n_sample); //function x valus for integral

	for (int i = 0; i < this->m; ++i) {
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
		//blaze::row(u, 2UL) = (K_z[0] * blaze::row(y, 3UL) + K_z[1] * blaze::row(y, 4UL)
		//	+ K_z[2] * blaze::row(y, 5UL)) / blaze::sum(K_z);
		blaze::row(u, 2UL) = 0.0; // force u_z to be zero
		blaze::submatrix(u_int, 0UL, i * this->bvp.n_sample, 3UL, this->bvp.n_sample) = u;
		blaze::subvector(x_int, i * this->bvp.n_sample, this->bvp.n_sample) = this->bvp.x_mesh[i];
	}

	// numerical integral
	auto pos = this->cumtrapz(x_int, u_int);
	blaze::row(pos, 2UL) += this->bc.points[0UL];

	// save shape
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
	blaze::DynamicVector<double, blaze::rowVector> height = blaze::subvector(x_int, 1UL, x_int.size() - 1)
		- blaze::subvector(x_int, 0UL, x_int.size() - 1);
	blaze::DynamicMatrix<double, blaze::rowMajor> base
		= blaze::submatrix(y_int, 0UL, 0UL, 3UL, y_int.columns() - 1)
		+ blaze::submatrix(y_int, 0UL, 1UL, 3UL, y_int.columns() - 1);

	// calculate area
	for (int i = 0; i < 3; ++i) {
		blaze::row(area, i) = blaze::row(base, i) * height * 0.5;
	}

	blaze::StaticVector<double, 3UL> cum_area;
	blaze::StaticMatrix<double, 3UL, 3UL> u_hat = { {0.0, -cum_area[2], cum_area[1]},
			{cum_area[2], 0.0, -cum_area[0]}, {-cum_area[1], cum_area[0], 0.0} };
	blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor> R = blaze::matexp(u_hat);
	blaze::DynamicMatrix<double, blaze::columnMajor> p_dot(3UL, x_int.size(), 0.0);

	// need double check
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
void CTR::SolveBC()
{
	blaze::StaticMatrix<double, 3UL, 3UL> BV; //calculate boundary value matrix
	BV = { {-CTR::tubes[0].GetLS(), -CTR::tubes[1].GetLS(), -CTR::tubes[2].GetLS()},
		{0, 0, 0}, {CTR::tubes[0].GetLC(), CTR::tubes[1].GetLC(), CTR::tubes[2].GetLC()} };
	BV += blaze::expand<3>(blaze::trans(this->TubeTran));
	this->bc.points = this->bc.GetUnique(BV); //boundary value points
	this->m = (int)this->bc.points.size() - 1; //number of CTR segments

	// determine the curvature at each section
	this->bc.curvatures.resize(m, 3UL, false); //resize variable
	this->bc.curvatures = 0;
	for (int i = 0; i < m; ++i) {
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
	blaze::StaticVector<int, 2UL, blaze::rowVector> interval = { 0, m - 1 }; //initial interval
	this->bc.intervals = blaze::expand<3>(interval);
	for (int i = 1; i < m; ++i) {
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
void CTR::GetFunc()
{
	// initialize variables
	blaze::StaticVector<double, 3UL> K_z;
	blaze::StaticMatrix<double, 3UL, 3UL> u_abs;

	//coefficient matrix for each segment
	blaze::StaticMatrix<double, 3UL, 3UL> C;

	// clear the previous container
	this->bvp.v_func.clear();

	for (int i = 0; i < this->m; ++i) {
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
			auto skewM = [&K_z, &u_abs]() {
				blaze::StaticMatrix<double, 3UL, 3UL> u_hat;
				u_hat = { {0.0, -K_z[2] * u_abs(2,2), K_z[1] * u_abs(1,1)},
					{K_z[2] * u_abs(2,2), 0.0, -K_z[0] * u_abs(0,0)},
					{-K_z[1] * u_abs(1,1), K_z[0] * u_abs(0,0), 0.0} };
				return u_hat;
			};
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

// display tube translation
std::string CTR::GetTran()
{
	std::string s;
	for (int i = 0; i < this->TubeTran.size(); ++i) {
		s += "\t";
		s += this->GetTran(i);
	}
	return s;
}

// display tube rotation
std::string CTR::GetRot()
{
	std::string s;
	for (int i = 0; i < this->TubeRot.size(); ++i) {
		s += "\t";
		s += this->GetRot(i);
	}
	return s;
}

// display tube translation
std::string CTR::GetTran(int i)
{
	std::string s;
	if (i >= 0 || i < this->TubeRot.size()) {
		s = std::to_string(round(this->TubeTran[i] * 1e6) * 1e-6);
	}
	return s;
}

// display tube rotation
std::string CTR::GetRot(int i)
{
	std::string s;
	if (i >= 0 || i < this->TubeTran.size()) {
		auto deg = this->TubeRot[i] * 180.0 / M_PI;
		s = std::to_string(round(deg * 1e3) * 1e-3);
	}
	return s;
}

// set tube translation
bool CTR::SetTran(int i, double val)
{
	if (i >= 0 || i < this->TubeTran.size()) {
		this->TubeTran[i] = val;
		return true;
	}
	else {
		return false;
	}
}

// set tube rotation
bool CTR::SetRot(int i, double val)
{
	if (i >= 0 || i < this->TubeRot.size()) {
		auto rad = val * M_PI / 180.0;
		this->TubeRot[i] = rad;
		return true;
	}
	else {
		return false;
	}
}

// display proximal end position
std::string CTR::GetProx()
{
	std::string s;
	auto pos = blaze::column(this->shapes[0UL], 0UL);
	for (int i = 0; i < this->shapes.size(); ++i) {
		s += "\t";
		s += std::to_string(round(pos[i] * 1e6) * 1e-6);
	}
	return s;
}

// display distal end position
std::string CTR::GetDist()
{
	std::string s;
	auto pos = blaze::column(this->shapes[0], this->shapes[0UL].columns() - 1);
	for (int i = 0; i < this->shapes.size(); ++i) {
		s += "\t";
		s += std::to_string(round(pos[i] * 1e6) * 1e-6);
	}
	return s;
}

// solve bvp function
void CTR::SolveBVP(int n_sample)
{
	this->bvp.n_sample = n_sample;
	this->bvp.m = this->m;

	this->bvp.x_mesh.clear();
	// assign x-mesh
	for (int i = 0; i < this->m; ++i) {
		this->bvp.x_mesh.push_back(blaze::linspace<blaze::rowVector>
			(n_sample, this->bc.points[i], this->bc.points[i + 1]));
	}

	// initialize bvp
	int it = 0;
	this->bvp.y_mesh = this->bvp.parareal(this->bc, this->TubeRot, this->bvp.guess_new);
	this->bvp.res_new = this->bvp.GetRes(this->bvp.y_mesh, this->bc);

	// initilize Ja
	double H = 1e-3; //jacobian small step
	if (this->bvp.isFirstIt) {
		blaze::DynamicVector<double> res_diff;
		for (int i = 0; i < 3; ++i) {
			auto guess_diff = this->bvp.guess_new;
			guess_diff[i] = this->bvp.guess_new[i] + H;
			auto y_mesh_diff = this->bvp.parareal(this->bc, this->TubeRot, guess_diff);
			res_diff = this->bvp.GetRes(y_mesh_diff, this->bc);
			blaze::column(this->bvp.Ja, i) = (res_diff - this->bvp.res_new) / H;
		}
		this->bvp.Ja_inv = this->pinv(this->bvp.Ja, 1e-9); // pinv with tol of 1e-9
		this->bvp.isFirstIt = false; // remove flag
	}

	// loop until error is small
	while (blaze::sum(blaze::abs(this->bvp.res_new)) > 1e-6) {
		if (it > 10) {
			// std::cout << this->bvp.guess_new << std::endl; //test code 
			this->bvp.guess_new *= -0.1;
			// std::cout << this->bvp.guess_new << std::endl; //test code 
			blaze::DynamicVector<double> res_diff;
			for (int i = 0; i < 3; ++i) {
				auto guess_diff = this->bvp.guess_new;
				guess_diff[i] = this->bvp.guess_new[i] + H * 10.0;
				auto y_mesh_diff = this->bvp.parareal(this->bc, this->TubeRot, guess_diff);
				res_diff = this->bvp.GetRes(y_mesh_diff, this->bc);
				blaze::column(this->bvp.Ja, i) = (res_diff - this->bvp.res_new) / H * 0.1;
			}
			this->bvp.Ja_inv = this->pinv(this->bvp.Ja, 1e-9); // pinv with tol of 1e-9
			it = 0; // reset counter
		}
		else if (it > 0) {
			// update Ja
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
		this->bvp.y_mesh = this->bvp.parareal(this->bc, this->TubeRot, this->bvp.guess_new);
		this->bvp.res_new = this->bvp.GetRes(this->bvp.y_mesh, this->bc);
		it++;
	}
}

// psudo matrix inverse
blaze::DynamicMatrix<double> CTR::pinv(const blaze::DynamicMatrix<double>& M, const double& tol)
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