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
	bool CalcIK_LM(int, Link );
	Matrix<double,3,3>Rodrigues(Matrix<double,3,1>, double);
	VectorXi FindRoute(int);
	Matrix<double,3,1>OmegaFromRotation( Matrix<double,3,3> );
	Matrix<double,6,1> CalcVWerr(Link, Link );
	void MoveJoints(VectorXi, MatrixXd);
	bool InverseKinematicsAll( Link, Link );
	bool InverseKinematics_LM_All( Link, Link );
};
#if 0
typedef Matrix<double,6,1> Vector6d;

/* == Kinematics == */
Matrix3d Rodrigues( Vector3d, double );
void CalcForwardKinematics( Link*, int);

VectorXi FindRoute( Link*, int);
Vector3d OmegaFromRotation( Matrix3d );
Matrix<double,6,1> CalcVWerr(Link, Link );
void MoveJoints(Link* link, VectorXi idx, VectorXd dq);
bool CalcInverseKinematics( Link*, int, Link );
bool CalcIK_LM(Link*, int, Link );
bool InverseKinematicsAll( Link*, Link, Link );
bool InverseKinematics_LM_All( Link*, Link, Link );
#endif


#endif
