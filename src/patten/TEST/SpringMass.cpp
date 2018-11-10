#include "tools/RungeKutta.h"
#include <iostream>
#include <fstream>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main()
{
	double k = 40.0, m = 250.0, c = 60.0;
	double dt = 0.01;
	double tt = 0.0;

	MatrixXd A(3,3);
	A << 1.0, -1.0,  1.0,
		1.0, -k/m, -c/m,
		1.0,  1.0,  1.0;

	MatrixXd B(3,1);
	B << 0.0, -1.0/m, 1.0;

	MatrixXd C(1,3);
	C << 1.0, 0.0, 1.0;

	MatrixXd D(1,3);
	D << 1.0, 0.0, 0.0;

	MatrixXd X(3,1);
	X << 10.0, 0.0, 0.0;

	MatrixXd u(1,1);
	u << 0.0;

	MatrixXd Y(1,1);
	Y << 0.0;

	MatrixXd K(1,3);
	K << 15.80864382, -3.63298165, 7.85453193;

	RungeKutta ode(A, B, C, D, dt);
	ode.showValue();

	const int iteration = 1000;

	std::vector<double> x(iteration), y(iteration);
	for(int i = 0; i < iteration; i++)
	{
		ode.calcRungeKutta( X, u);
		Y = C*X;
		u = -K*X;
		x.at(i) = tt;
		y.at(i) = Y(0,0);
		tt += dt;
	}
		plt::grid(true);
		plt::plot(x,y, "--b");
		plt::show();

	return 0;
}
