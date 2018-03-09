#include "kinematics.h"

/* === function === */

Matrix3d Rodrigues( Vector3d a, double q )
{
	double S = sin(q), C = cos(q);
	Matrix3d R;
	
	R(0,0) = C + pow(a(0),2) * (1-C);
	R(0,1) = a(0) * a(1) * (1-C) - a(2) * S;
	R(0,2) = a(0) * a(2) * (1-C) + a(1) * S;
	R(1,0) = a(1) * a(0) * (1-C) + a(2) * S;
	R(1,1) = C + pow(a(1),2) * (1-C);
	R(1,2) = a(1) * a(2) * (1-C) - a(0) * S;
	R(2,0) = a(2) * a(0) * (1-C) - a(1) * S;
	R(2,1) = a(2) * a(1) * (1-C) + a(0) * S;
	R(2,2) = C + pow(a(2),2) * (1-C);

	return R;

}

void CalcForwardKinematics( Link* link, int j)
{
	int i;
	if(j == -1) return;
	if(j != 0)
	{
		i = link[j].parent;
	
		link[j].p = link[i].p + link[i].R * link[j].b;
		link[j].R = link[i].R * Rodrigues( link[j].a, link[j].q );
	}

	CalcForwardKinematics( link, link[j].sister);
	CalcForwardKinematics( link, link[j].child);
}

VectorXi FindRoute( Link* link, int rootlink)
{
	int flag = 0;
	VectorXi idx;

	if(rootlink == LLEG_J5)
	{
		flag = RLEG_J5;
		rootlink = RLEG_J5;
	}
	else
	{
		flag = 0;
	}

	idx.resize(rootlink);	

	int link_num = rootlink;
	while(link_num != 0)
	{
		idx(link_num-1) = link_num + flag;
		link_num = link[link_num].parent;
	}
		
	return idx;
}


Vector3d OmegaFromRotation( Matrix3d R )
{
	double alpha = ( R(0,0) + R(1,1) + R(2,2) - 1.0) / 2.0;
	if(fabs(alpha - 1.0) < 1.0e-6)
	{
		return Vector3d::Zero();
	}
	else
	{
		double th = acos(alpha);
		double s  = sin(th);
		
		if(s< numeric_limits<double>::epsilon())
		{
			return Vector3d( sqrt((R(0,0)+1)*0.5)*th, sqrt((R(1,1)+1)*0.5)*th, sqrt((R(2,2)+1)*0.5)*th );
		}
		double k = -0.5 * th / s;
		return Vector3d((R(1,2)-R(2,1)) * k, (R(2,0)-R(0,2)) * k, (R(0,1)-R(1,0)) * k);
	}

}
Matrix<double,6,1> CalcVWerr(Link Cref, Link Cnow)
{
  Matrix<double,3,1> perr = Cref.p - Cnow.p;
	Matrix<double,3,3> Rerr = Cnow.R.transpose() * Cref.R;
	Matrix<double,3,1> werr = Cnow.R * OmegaFromRotation(Rerr);
	Matrix<double,6,1> err;
	err << perr,werr;
	return err;
}

bool CalcInverseKinematics( Link* link, int to, Link target)
{
	MatrixXd J;
	VectorXd dq;
	Matrix<double,6,1> err;

	const double lambda = 0.5;
	const int iteration = 100;
	VectorXi idx = FindRoute( link, to);
	const int jsize = idx.size();
	J.resize(6,jsize); dq.resize(jsize);

	for(int i = 0; i < iteration; i++)
	{
		J = CalcJacobian( link, idx );
		err = CalcVWerr( target, link[to]);
		if(err.norm() < eps)
		{
			return true;
			break;
		}

		dq = J.inverse() * err * lambda;
		MoveJoints(link, idx, dq);
		CalcForwardKinematics(link, CC);

	}
		return false;
}

bool InverseKinematicsAll( Link* link, Link Target_R, Link Target_L )
{
	if(!CalcInverseKinematics( link, RLEG_J5, Target_R ) || !CalcInverseKinematics( link, LLEG_J5, Target_L ))
	{
		cout << "IK Calculation failure" << endl;
		return false;
	}

	return true;
}

void MoveJoints(Link* link, VectorXi idx, VectorXd dq)
{
	for(int n = 0; n < idx.size(); n++)
	{
		int j = idx(n);
		link[j].q = link[j].q + dq(n);
	}
}

bool CalcIK_LM(Link* link, int to, Link target)
{
	MatrixXd J, Jh;
	VectorXi idx = FindRoute( link, to);
	VectorXd dq;
	double wn_pos = 1/0.3;
	double wn_ang = 1/(2*pi);
	double lambda, Ek, Ek2;
	Matrix< double, 6,1 > we, err, gerr;
	we << wn_pos, wn_pos, wn_pos, wn_ang, wn_ang, wn_ang;
	Matrix< double, 6,6 >We = we.array().matrix().asDiagonal();
	MatrixXd Wn = MatrixXd::Identity(idx.size(),idx.size());

	CalcForwardKinematics( link, CC);
	err = CalcVWerr( target, link[to]);
	Ek = err.transpose() * We * err;

	for(int i = 0; i < 10; i++)
	{

		J = CalcJacobian( link, idx );
		lambda = Ek + 0.002;
		Jh   = J.transpose() * We * J + Wn * lambda; //Hk + wn
		gerr = J.transpose() * We * err; // gk
		
		dq = Jh.inverse() * gerr; // new
		MoveJoints(link, idx, dq);

		Ek2 = gerr.transpose() * We * gerr;

		if(Ek2 < 1E-12)
		{
			return true;
			break;
		}
		else if( Ek2 < Ek )
		{
			Ek = Ek2;
		}
		else
		{
			MoveJoints(link, idx, -dq);
			return true;
			break;
		}
	}
	return false;
}

bool InverseKinematics_LM_All( Link* link, Link Target_R, Link Target_L )
{
	if(!CalcIK_LM( link, RLEG_J5, Target_R ) || !CalcIK_LM( link, LLEG_J5, Target_L ))
	{
		cout << "IK Calculation failure" << endl;
   	return false;
	}
	return true;
}

