#include "tools/RungeKutta.h"
#include <iostream>
#include <fstream>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main()
{
	double Zc = 0.8;
	double Acc_g = 9.81;
	double omega = Acc_g/Zc;
	double omega2 = pow(omega,2);

	double dt = 0.01;
	double tt = 0.0;

	MatrixXd A(2,2);
	A << 0.0, 1.0,
		omega2, 0.0;


	MatrixXd B(2,1);
	B << 0.0, -omega2;

	MatrixXd C(1,2);
	C << 1.0, 0.0;

	MatrixXd D(1,2);
	D << 0.0, 0.0;

	MatrixXd X(2,1);
	X << 0.0, 0.0;

	MatrixXd u(1,1);
	u << 0.0;

	MatrixXd Y(1,1);
	Y << 0.0;
/*
	MatrixXd K(1,3);
	K << 15.80864382, -3.63298165, 7.85453193;
*/
	RungeKutta ode(A, B, C, D, dt);
	ode.showValue();

	const int iteration = 1000;

	std::vector<double> x(iteration), y(iteration);
	for(int i = 0; i < iteration; i++)
	{
		ode.calcRungeKutta( X, u);
		Y = C*X;

		x.at(i) = tt;
		y.at(i) = Y(0,0);
		tt += dt;
	}
		plt::grid(true);
		plt::plot(x,y, "--b");
		plt::show();

	return 0;
}
