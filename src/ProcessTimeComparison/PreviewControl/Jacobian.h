#ifndef _JACOBIAN_H_
#define _JACOBIAN_H_

#include <iostream>
#include <Eigen/Core>

#include "Link.h"

using namespace Eigen;
using namespace MotionControl;

Matrix<double,3,1> calcMC(Link *ulink, int rootlink);
double calcTotalMass(Link *ulink, int rootlink);
Matrix<double,3,1> calcCoM(Link *ulink);
MatrixXd calcJacobian(Link *ulink, std::vector<int> idx);
MatrixXd calcJacobian2(Link *ulink, std::vector<int> idx);
MatrixXd calcJacobian_GankenKun(Link *ulink, std::vector<int> idx);

#endif
