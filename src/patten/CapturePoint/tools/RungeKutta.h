#ifndef _RUNGEKUTTA_H_
#define _RUNGEKUTTA_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
using namespace Eigen;

class RungeKutta
{
	private:
		MatrixXd A;
		MatrixXd b;
		MatrixXd c;
		MatrixXd d;
		MatrixXd u;
		MatrixXd x;
		double dt;
		std::ofstream ofs;
		
	public:
		RungeKutta(MatrixXd in_A, MatrixXd in_b, MatrixXd in_c, MatrixXd in_d, double in_dt);
		~RungeKutta();
		void showValue();
		void calcRungeKutta( MatrixXd &x, MatrixXd u);
};

#endif
