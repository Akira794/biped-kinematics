#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "link.h"
#include "jacobian.h"

class Kinematics
{
public:
	struct Link *ulink;
public:
	Kinematics(Link *ulink)
	{
		this->ulink = ulink;
	}
	void CalcForwardKinematics(int);
	bool CalcInverseKinematics(int, Link);
	void MoveJoints( vector<int>,MatrixXd );
	bool CalcLMInverseKinematics(int, Link );
	vector<int>FindRoute(int);
	Matrix<double,3,3>RotationFromRPY( double, double, double);

	Matrix<double,3,3>Rodrigues(Matrix<double,3,1>, double);
	Matrix<double,3,1>OmegaFromRotation( Matrix<double,3,3> );
	Matrix<double,6,1> CalcVWerr(Link, Link );

};

#endif
