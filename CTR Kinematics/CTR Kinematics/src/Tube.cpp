#include "Tube.h"

//nitinol properties
const double Tube::E = 58e9;
const double Tube::G = 21.5e9;
const double Tube::v = 0.5 * Tube::E / Tube::G - 1.0;

//constructor
//Tube::Tube(){};

//set tube dimensions
void Tube::SetDimension(double ID_val, double OD_val, double ls_val, double lc_val, double r_val)
{
	//check bending radius
	if (r_val <= 0.0) {
		std::cout << "Tube bending raduis is too small!" << std::endl;
		return;
	}
	this->ID = ID_val;
	this->OD = OD_val;
	this->l_s = ls_val;
	this->l_c = lc_val;
	this->r = r_val;

	this->I = M_PI_4 * (pow(OD_val * 0.5, 4.0) - pow(ID_val * 0.5, 4.0));
	this->J = M_PI_2 * (pow(OD_val * 0.5, 4.0) - pow(ID_val * 0.5, 4.0));
	this->u_abs = 1.0 / r_val;
	this->u_pre = { this->u_abs, 0.0, 0.0 };
	this->K_xy = this->E * this->I;
	this->K_z = this->G * this->J;
	this->K = { {this->K_xy, 0, 0},{0, this->K_xy, 0},{0, 0, this->K_z} };
}

//get E value
double Tube::GetE() { return Tube::E; }

//get G value
double Tube::GetG() { return Tube::G; }

//get v value
double Tube::GetV() { return Tube::v; }

//get ID
double Tube::GetID() const { return this->ID; }

//get OD
double Tube::GetOD() const { return this->OD; }

//get straight length
double Tube::GetLS() const { return this->l_s; }

//get curved length
double Tube::GetLC() const { return this->l_c; }

//get radius
double Tube::GetR() const { return this->r; }

//get I value
double Tube::GetI() const { return this->I; }

//get J value
double Tube::GetJ() const { return this->J; }

//get u_abs value
double Tube::GetUabs() const { return this->u_abs; }

//get u vector
blaze::StaticVector<double, 3UL> Tube::GetUpre() const { return this->u_pre; }

//get K_xy value
double Tube::GetKxy() const { return this->K_xy; }

//get K_z value
double Tube::GetKz() const { return this->K_z; }

//get K matrix
blaze::StaticMatrix<double, 3UL, 3UL> Tube::GetK() const { return this->K; }