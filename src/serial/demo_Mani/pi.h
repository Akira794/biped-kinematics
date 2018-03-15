#ifndef __PI_H__
#define __PI_H__

#include "demoManipLink.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

Matrix3d RotationFromRPY( double, double, double);
double Rad2Deg( double );
double Deg2Rad( double );

#endif
