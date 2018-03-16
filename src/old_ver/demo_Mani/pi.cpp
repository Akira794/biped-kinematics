#include "pi.h"

/*
Matrix3d RotationFromRPY( double r, double p, double y)
{
	Matrix3d R;
	R(0,0) = cos(y)*cos(p);
	R(0,1) = cos(y)*sin(p)*sin(r)-sin(y)*cos(r);
	R(0,2) = sin(y)*sin(r)+cos(y)*sin(p)*cos(r);
	R(1,0) = sin(y)*cos(p);
	R(1,1) = cos(y)*cos(r)+sin(y)*sin(p)*sin(r);
	R(1,2) = sin(y)*sin(p)*cos(r)-cos(y)*sin(r);
	R(2,0) = -sin(p);
	R(2,1) = cos(p)*sin(r);
	R(2,2) = cos(p)*cos(r);

	return R;
}
*/

double Rad2Deg( double rad )
{
	return (180/pi) * rad;
}

double Deg2Rad( double deg )
{
	return (pi/180) * deg;
}

