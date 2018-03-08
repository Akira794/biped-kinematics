#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "link.h"
#include "jacobian.h"

typedef Matrix<double,6,1> Vector6d;

/* == Kinematics == */
Matrix3d Rodrigues( Vector3d, double );
void CalcForwardKinematics( Link*, int);

VectorXi FindRoute( Link*, int);
Vector3d OmegaFromRotation( Matrix3d );
Vector6d CalcVWerr(Link, Link );
bool CalcInverseKinematics( Link*, int, Link );
bool InverseKinematicsAll( Link*, Link, Link );

#if 0
void CalcIK_LM(Link*, int, Link );
#endif

#endif
