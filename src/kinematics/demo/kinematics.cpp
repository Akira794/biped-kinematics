#include "kinematics.h"

/* === function === */

Matrix<double,3,3> Kinematics::RotationFromRPY( double r, double p, double y)
{
	Matrix<double,3,3> R;
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

Matrix<double,3,3>Kinematics::Rodrigues( Matrix<double,3,1> a, double q )
{
	return AngleAxisd(q,a).toRotationMatrix();
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

vector<int> Kinematics::FindRoute(int to)
{
	vector<int> idx;
	int link_num = to;
	while( link_num != 0)
	{
		idx.push_back(link_num);
		link_num = ulink[link_num].parent;
	}
	reverse(idx.begin(), idx.end());
	return idx;
}

Matrix<double,6,1> Kinematics::calcVWerr(Link Cref, Link Cnow)
{
  Matrix<double,3,1> perr = Cref.p - Cnow.p;
	Matrix<double,3,3> Rerr = Cnow.R.transpose() * Cref.R;
	Matrix<double,3,1> werr = Cnow.R * OmegaFromRotation(Rerr);
	Matrix<double,6,1> err;
	err << perr,werr;
	return err;
}

void Kinematics::calcForwardKinematics( int j)
{
	int i;
	if(j == -1) return;
	if(j != 0)
	{
		i = ulink[j].parent;
	
		ulink[j].p = ulink[i].p + ulink[i].R * ulink[j].b;
		ulink[j].R = ulink[i].R * Rodrigues( ulink[j].a, ulink[j].q );
	}

	calcForwardKinematics(ulink[j].sister);
	calcForwardKinematics(ulink[j].child);
}

bool Kinematics::calcInverseKinematics(int to, Link target)
{
	MatrixXd J,dq;
	Matrix<double,6,1> err;

	const double lambda = 0.5;
	const int iteration = 100;
	vector<int> idx = FindRoute(to);
	const int jsize = idx.size();
	J.resize(6,jsize); dq.resize(jsize,1);

	for(int i = 0; i < iteration; i++)
	{
		J = calcJacobian( ulink, idx );
		err = calcVWerr( target, ulink[to]);
		if(err.norm() < eps)
		{
			return true;
			break;
		}

		dq = J.inverse() * err * lambda;
		MoveJoints(idx, dq);
		calcForwardKinematics(BASE);

	}
		return false;
}

bool Kinematics::InverseKinematicsAll(Link Target_R, Link Target_L )
{
	if(!calcInverseKinematics(RLEG_J5, Target_R ) || !calcInverseKinematics(LLEG_J5, Target_L ))
	{
		cout << "IK Calculation failure" << endl;
		return false;
	}

	return true;
}

void Kinematics::MoveJoints( vector<int> idx, MatrixXd dq )
{
	for(int n=0; n<idx.size(); n++)
	{
		int j = idx[n];
		ulink[j].q += dq(n);
	}
}

bool Kinematics::calcLMInverseKinematics(int to, Link target)
{
#if 1
	MatrixXd J, Jh, dq;
	vector<int> idx = FindRoute(to);
	double wn_pos = 1/0.3;
	double wn_ang = 1/(2*pi);
	double lambda, Ek, Ek2;
	Matrix< double, 6,1 > we, err, gerr;
	we << wn_pos, wn_pos, wn_pos, wn_ang, wn_ang, wn_ang;
	Matrix< double, 6,6 >We = we.array().matrix().asDiagonal();
	MatrixXd Wn = MatrixXd::Identity(idx.size(),idx.size());

	calcForwardKinematics(BASE);
	err = calcVWerr( target, ulink[to]);
	Ek = err.transpose() * We * err;

	for(int i = 0; i < 10; i++)
	{

		J = calcJacobian( ulink, idx );
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
	if(!calcLMInverseKinematics( link, RLEG_J5, Target_R ) || !calcLMInverseKinematics( link, LLEG_J5, Target_L ))
	{
		cout << "IK Calculation failure" << endl;
   	return false;
	}
	return true;
#endif
}

