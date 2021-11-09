#include "BVP.h"

// constructor
BVP::BVP() : isFirstIt(true)
{
	this->guess_new = { 0.0, 0.0, 0.0 };
}

// determine Jacobian for bvp
void BVP::GetJa(const double& H)
{

}

// determine bvp residue
blaze::StaticVector<double, 3UL> BVP::GetRes(
	const std::vector<blaze::DynamicMatrix<double, blaze::columnMajor>>& mesh,
	const BC& bc)
{
	blaze::StaticVector<double, 3UL> res;
	for (int i = 0; i < 3; ++i) {
		res[i] = mesh[bc.intervals(i, 1UL)](i + 3, mesh[bc.intervals(i, 1UL)].columns() - 1);
	}
	return res;
}

// parareal algorithm
std::vector<blaze::DynamicMatrix<double, blaze::columnMajor>> BVP::parareal(
	const BC& bc,
	const blaze::StaticVector<double, 3UL>& rot,
	const blaze::StaticVector<double, 3UL>& guess)
{
	// std::cout << "-----------parareal alg called------------" << std::endl; //test code
	// initialize left value
	blaze::DynamicMatrix<double, blaze::columnMajor> xL(6UL, this->m, 0.0);
	for (int i = 0; i < 3; ++i) {
		xL(i, bc.intervals(i, 0)) = rot[i];
		xL(i + 3, bc.intervals(i, 0)) = guess[i];
	}

	// initalize right value
	blaze::DynamicVector<double, blaze::columnVector> x_RK1;
	for (int i = 0; i < this->m - 1; ++i) {
		//RK1 formula
		x_RK1 = blaze::column(xL, i) + (this->x_mesh[i + 1][0] - this->x_mesh[i][0])
			* (this->v_func[i](blaze::column(xL, i)));
		blaze::column(xL, i + 1) = x_RK1 * blaze::column(bc.types, i + 1)
			+ blaze::column(xL, i + 1) * !(blaze::column(bc.types, i + 1));
	}
	blaze::DynamicMatrix<double, blaze::columnMajor> xR_G(6UL, this->m, 0.0);
	std::vector<blaze::DynamicMatrix<double, blaze::columnMajor>> xR_F_mesh(this->m);
	std::vector<std::future<void>> fu(this->m);

	while (true) {
		//// normal for
		//for (int i = 0; i < this->m - 1; ++i) {
		//	blaze::column(xR_G, i) = blaze::column(xL, i) + (this->x_mesh[i + 1][0] - this->x_mesh[i][0])
		//		* (this->v_func[i](blaze::column(xL, i)));
		//	xR_F_mesh[i] = this->RK4(this->x_mesh[i], blaze::column(xL, i), this->v_func[i]);
		//}

		// parallel for
		for (int i = 0; i < this->m - 1; ++i) {
			fu[i] = std::async(std::launch::async, [this, &xR_G, &xL, &xR_F_mesh](int i) {
				blaze::column(xR_G, i) = blaze::column(xL, i)
					+ (this->x_mesh[i + 1][0] - this->x_mesh[i][0])
					* (this->v_func[i](blaze::column(xL, i)));
				xR_F_mesh[i] = this->RK4(this->x_mesh[i], blaze::column(xL, i), this->v_func[i]);
				}, i);
		}
		for (int i = 0; i < this->m - 1; ++i) {
			fu[i].wait();
		}

		xR_F_mesh[this->m - 1] = 0.0 * xR_F_mesh[0]; // necessary to initialize the last matrix as well
		auto xR_F = this->squeeze3(xR_F_mesh); //squeeze function
		auto xR_delta = xR_F - xR_G;
		auto xL_old = xL;

		for (int i = 0; i < this->m - 1; ++i) {
			x_RK1 = blaze::column(xL, i) + (this->x_mesh[i + 1][0] - this->x_mesh[i][0])
				* (this->v_func[i](blaze::column(xL, i)));
			blaze::column(xL, i + 1) = (x_RK1 + blaze::column(xR_delta, i)) * blaze::column(bc.types, i + 1)
				+ blaze::column(xL, i + 1) * !(blaze::column(bc.types, i + 1));
		}

		if (blaze::norm(xL_old - xL) < 1e-6) {
			// additional loop with parallel computing
			for (int i = 0; i < this->m; ++i) {
				fu[i] = std::async(std::launch::async, [this, &xL, &xR_F_mesh](int i) {
					xR_F_mesh[i] = this->RK4(this->x_mesh[i], blaze::column(xL, i), this->v_func[i]);
					}, i);
			}
			for (int i = 0; i < this->m; ++i) {
				fu[i].wait();
			}
			return xR_F_mesh;
		}
	}
}


// RK4 algorithm
blaze::DynamicMatrix<double, blaze::columnMajor> BVP::RK4(
	const blaze::DynamicVector<double, blaze::rowVector>& x,
	const blaze::DynamicVector<double>& y_init,
	const bvfun& func)
{
	blaze::DynamicMatrix<double, blaze::columnMajor> y(y_init.size(), x.size(), 0.0);
	blaze::column(y, 0UL) = y_init;

	for (int i = 0; i < x.size() - 1; ++i) {
		auto h = x[i + 1] - x[i];
		blaze::DynamicVector<double> k1 = func(blaze::column(y, i));
		blaze::DynamicVector<double> k2 = func(blaze::column(y, i) + k1 * 0.5 * h);
		blaze::DynamicVector<double> k3 = func(blaze::column(y, i) + k2 * 0.5 * h);
		blaze::DynamicVector<double> k4 = func(blaze::column(y, i) + k3 * h);
		blaze::column(y, i + 1) = blaze::column(y, i) + (k1 + k2 * 2 + k3 * 2 + k4) * h / 6.0;
	}
	return y;
}

// 3d-matrix squeeze function
blaze::DynamicMatrix<double, blaze::columnMajor> BVP::squeeze3(
	const std::vector<blaze::DynamicMatrix<double, blaze::columnMajor>>& M3D)
{
	int m = M3D.size();
	blaze::DynamicMatrix<double, blaze::columnMajor> M2D(6UL, m, 0.0);
	for (int i = 0; i < m; ++i) {
		blaze::column(M2D, i) = blaze::column(M3D[i], M3D[i].columns() - 1);
	}
	return M2D;
}