#pragma once
#define _USE_MATH_DEFINES //use predefined math constants

#include <iostream>
#include <cmath>
#include <blaze/Math.h>

class Tube
{
public:
	//constructor
	//Tube();

	//add tube dimensions
	void SetDimension(double ID_val, double OD_val, double ls_val, double lc_val, double r_val);

	//get E value
	static double GetE();

	//get G value
	static double GetG();

	//get v value
	static double GetV();

	//get ID
	double GetID() const;

	//get OD
	double GetOD() const;

	//get straight length
	double GetLS() const;

	//get curved length
	double GetLC() const;

	//get radius
	double GetR() const;

	//get I value
	double GetI() const;

	//get J value
	double GetJ() const;

	//get u_abs value
	double GetUabs() const;

	//get u vector
	blaze::StaticVector<double, 3UL> GetUpre() const;

	//get K_xy value
	double GetKxy() const;

	//get K_z value
	double GetKz() const;

	//get K matrix
	blaze::StaticMatrix<double, 3UL, 3UL> GetK() const;

private:
	const static double E; //Young's modulus
	const static double G; //shear modulus
	const static double v; //Poisson's ratio

	double ID;  //tube inner diameter
	double OD; //tube outer diameter
	double l_s; //straight section length
	double l_c; //curved section length
	double r; //bend radius at curved section

	double I; //moment of inertia
	double J; //polar moment of inertia;
	double u_abs; //tube pre-curvature vector length
	blaze::StaticVector<double, 3UL> u_pre; //tube pre-curvature vector
	double  K_xy; //tube bending stiffness constant on x & y axis
	double K_z; //tube bending stiffness constant on z axis
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> K; //tube bending stiffness matrix
};