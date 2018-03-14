#include "kinematics.h"

/* === function === */

Matrix<double,3,3>Kinematics::Rodrigues( Matrix<double,3,1> a, double q )
{
	return AngleAxisd(q,a).toRotationMatrix();
}

void Kinematics::CalcForwardKinematics( int j)
{
	int i;
	if(j == -1) return;
	if(j != 0)
	{
		i = ulink[j].parent;
	
		ulink[j].p = ulink[i].p + ulink[i].R * ulink[j].b;
		ulink[j].R = ulink[i].R * Rodrigues( ulink[j].a, ulink[j].q );
	}

	CalcForwardKinematics(ulink[j].sister);
	CalcForwardKinematics(ulink[j].child);
}

VectorXi Kinematics::FindRoute( int rootlink)
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
		link_num = ulink[link_num].parent;
	}
		
	return idx;
}


Matrix<double,3,1> Kinematics::OmegaFromRotation( Matrix<double,3,3> R )
{
	double alpha = ( R(0,0) + R(1,1) + R(2,2) - 1.0) / 2.0;
	Matrix<double,3,1> vector_R(Matrix<double,3,1>::Zero());
	if(fabs(alpha - 1.0) < 1.0e-6)
	{
		return  Matrix<double,3,1>::Zero();
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
		vector_R(0,0) = (R(1,2)-R(2,1)) * k;
		vector_R(1,0) = (R(2,0)-R(0,2)) * k;
		vector_R(2,0) = (R(0,1)-R(1,0)) * k;
		return vector_R;
	}

}
Matrix<double,6,1> Kinematics::CalcVWerr(Link Cref, Link Cnow)
{
  Matrix<double,3,1> perr = Cref.p - Cnow.p;
	Matrix<double,3,3> Rerr = Cnow.R.transpose() * Cref.R;
	Matrix<double,3,1> werr = Cnow.R * OmegaFromRotation(Rerr);
	Matrix<double,6,1> err;
	err << perr,werr;
	return err;
}

bool Kinematics::CalcInverseKinematics(int to, Link target)
{
	MatrixXd J,dq;
	Matrix<double,6,1> err;

	const double lambda = 0.5;
	const int iteration = 100;
	VectorXi idx = FindRoute(to);
	const int jsize = idx.size();
	J.resize(6,jsize); dq.resize(jsize,1);

	for(int i = 0; i < iteration; i++)
	{
		J = CalcJacobian( ulink, idx );
		err = CalcVWerr( target, ulink[to]);
		if(err.norm() < eps)
		{
			return true;
			break;
		}

		dq = J.inverse() * err * lambda;
		MoveJoints(idx, dq);
		CalcForwardKinematics(CC);

	}
		return false;
}

bool Kinematics::InverseKinematicsAll(Link Target_R, Link Target_L )
{
	if(!CalcInverseKinematics(RLEG_J5, Target_R ) || !CalcInverseKinematics(LLEG_J5, Target_L ))
	{
		cout << "IK Calculation failure" << endl;
		return false;
	}

	return true;
}

void Kinematics::MoveJoints(VectorXi idx, MatrixXd dq)
{
	for(int n = 0; n < idx.size(); n++)
	{
		int j = idx(n);
		ulink[j].q = ulink[j].q + dq(n);
	}
}

bool Kinematics::CalcIK_LM(int to, Link target)
{
#if 1
	MatrixXd J, Jh, dq;
	VectorXi idx = FindRoute(to);
	double wn_pos = 1/0.3;
	double wn_ang = 1/(2*pi);
	double lambda, Ek, Ek2;
	Matrix< double, 6,1 > we, err, gerr;
	we << wn_pos, wn_pos, wn_pos, wn_ang, wn_ang, wn_ang;
	Matrix< double, 6,6 >We = we.array().matrix().asDiagonal();
	MatrixXd Wn = MatrixXd::Identity(idx.size(),idx.size());

	CalcForwardKinematics(CC);
	err = CalcVWerr( target, ulink[to]);
	Ek = err.transpose() * We * err;

	for(int i = 0; i < 10; i++)
	{

		J = CalcJacobian( ulink, idx );
		lambda = Ek + 0.002;
		Jh   = J.transpose() * We * J + Wn * lambda; //Hk + wn
		gerr = J.transpose() * We * err; // gk
		
		dq = Jh.inverse() * gerr; // new qk
		MoveJoints(idx, dq);

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
			MoveJoints(idx, -dq);
			return true;
			break;
		}
	}
	return false;
#endif
}

bool Kinematics::InverseKinematics_LM_All( Link Target_R, Link Target_L )
{
#if 0
	if(!CalcIK_LM( link, RLEG_J5, Target_R ) || !CalcIK_LM( link, LLEG_J5, Target_L ))
	{
		cout << "IK Calculation failure" << endl;
   	return false;
	}
	return true;
#endif
}

