#ifndef __JACOBIAN_H__
#define __JACOBIAN_H__

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "link.h"

using namespace std;
using namespace Eigen;

MatrixXd CalcJacobian( Link*, vector<int> idx );
#endif
