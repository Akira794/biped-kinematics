#include "RungeKutta.h"



RungeKutta::RungeKutta(MatrixXd in_A, MatrixXd in_b, MatrixXd in_c, MatrixXd in_d, double in_dt)
{ A = in_A; b = in_b; c = in_c; d = in_d; dt = in_dt;}

RungeKutta::~RungeKutta(){}

void RungeKutta::calcRungeKutta( MatrixXd &x, MatrixXd u )
{
		MatrixXd k1 = A*x + b*u;
		MatrixXd k2 = A*(x + 0.5*k1*dt) + b*u;
		MatrixXd k3 = A*(x + 0.5*k2*dt) + b*u;
		MatrixXd k4 = A*(x + k3*dt) + b*u;
		MatrixXd k = (k1 + 2.0*k2 + 2.0*k3 + k4)*dt / 6.0;
		x = x + k;
}

void RungeKutta::showValue()
{
	std::cout << "A=" << std::endl << A << std::endl;
	std::cout << std::endl;
	std::cout << "b=" << std::endl << b << std::endl;
	std::cout << std::endl;
	std::cout << "c=" << std::endl << c << std::endl;
	std::cout << std::endl;
	std::cout << "d=" << std::endl << d << std::endl;
	std::cout << std::endl;
	std::cout << "dt=" << std::endl << dt << std::endl;

}

/*
void RungeKutta::calcODE( MatrixXd x, MatrixXd u, MatrixXd k, const int iteration )
{
	ofs.open("outdata.csv", ios::out);
	for(int i = 0; i < iteration; i++)
	{
		ode.calcRungeKutta(x,u);
		
	}
}
*/
