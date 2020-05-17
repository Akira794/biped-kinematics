#include "Kinematics.h"
#include <Eigen/SVD>

using namespace MotionControl;

Matrix<double,3,3> Kinematics::Rodrigues(Matrix<double,3,1> a, double q)
{
	return AngleAxisd(q,a).toRotationMatrix();
}

Matrix<double,3,1> Kinematics::rot2omega(Matrix<double,3,3> R)
{
	double alpha = (R(0,0)+R(1,1)+R(2,2)-1)/2;
	double th;
	Matrix<double,3,1> vector_R(Matrix<double,3,1>::Zero());

	if(fabs(alpha-1) < eps)
		return Matrix<double,3,1>::Zero();

	th = std::cos(alpha);
	vector_R << R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1);
	return 0.5*th/std::sin(th)*vector_R;
}

Eigen::Matrix<double,3,3> Kinematics::computeMatrixFromAngles(double roll, double pitch, double yaw)
{
	Eigen::Matrix<double,3,3> R;

	R(0,0) = std::cos(pitch) * std::cos(yaw) - std::sin(roll) * std::sin(pitch) * std::sin(yaw);
	R(0,1) = -std::cos(roll) * std::sin(yaw);
	R(0,2) = std::sin(roll) * std::cos(yaw) + std::sin(roll) * std::cos(pitch) * std::sin(yaw);
	R(1,0) = std::cos(pitch) * std::sin(yaw) + std::sin(roll) * std::sin(pitch) * std::cos(yaw);
	R(1,1) = std::cos(roll) * std::cos(yaw);
	R(1,2) = std::sin(pitch) * std::sin(yaw) - std::sin(roll) * std::cos(pitch) * std::cos(yaw);
	R(2,0) = -std::cos(roll) * std::sin(pitch);
	R(2,1) = std::sin(roll);
	R(2,2) = std::cos(roll) * std::cos(pitch);

	return R;
}

void Kinematics::computeAnglesFromMatrix(Eigen::Matrix<double,3,3> R, double &roll, double &pitch, double &yaw)
{
	float threshold = 0.001;
	if(abs(R(2,1) - 1.0) < threshold){ // R(2,1) = std::sin(x) = 1の時
		roll 	= M_PI / 2;
		pitch = 0;
		yaw 	= std::atan2(R(1,0), R(0,0));
	}else if(abs(R(2,1) + 1.0) < threshold){ // R(2,1) = std::sin(x) = -1の時
		roll	= - M_PI / 2;
		pitch = 0;
		yaw		= std::atan2(R(1,0), R(0,0));
	}else{
		roll	= std::sin(R(2,1));
		pitch	= std::atan2(-R(2,0), R(2,2));
		yaw		= std::atan2(-R(0,1), R(1,1));
	}
}

vector<int> Kinematics::FindRoute(int to)
{
	vector<int> idx;
	int link_num = to;

	while(link_num != 0)
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
	Matrix<double,3,3> Rerr = Cref.R - Cnow.R;
	Matrix<double,3,1> werr = Cnow.R * rot2omega(Rerr);
	Matrix<double,6,1> err;

	err << perr,werr;
	return err;
}

void Kinematics::calcForwardKinematics(int rootlink)
{
	if(rootlink == -1) return;
	if(rootlink != 0)
	{
		int parent = ulink[rootlink].parent;
		ulink[rootlink].p = ulink[parent].R * ulink[rootlink].b + ulink[parent].p;
		ulink[rootlink].R = ulink[parent].R * Rodrigues(ulink[rootlink].a, ulink[rootlink].q);
	}
	calcForwardKinematics(ulink[rootlink].sister);
	calcForwardKinematics(ulink[rootlink].child);
}

// 逆運動学
bool Kinematics::calcInverseKinematics(int to, Link target)
{
	Matrix<double,6,6> J;
	Matrix<double,6,1> dq, err;

	ColPivHouseholderQR<MatrixXd> QR; //QR分解?
	const double lambda = 0.5;
	const int iteration = 100;

	calcForwardKinematics(CC);
	vector<int> idx = FindRoute(to);

	for(int n=0;n<iteration;n++){
		J = calcJacobian(ulink, idx);
		err = calcVWerr(target, ulink[to]);
		if(err.norm() < eps) return true;
		dq = lambda * (J.inverse() * err);
		for(size_t nn=0;nn<idx.size();nn++){
			int j = idx[nn];
			ulink[j].q += dq(nn);
		}
		calcForwardKinematics(CC);
	}
	return false;
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
	MatrixXd J, Jh, dq;
	vector<int> idx = FindRoute(to);
	double wn_pos = 1/0.3;
	double wn_ang = 1/(2*pi);
	double lambda, Ek, Ek2;
	Matrix< double, 6,1 > we, err, gerr;
	we << wn_pos, wn_pos, wn_pos, wn_ang, wn_ang, wn_ang;
	Matrix< double, 6,6 >We = we.array().matrix().asDiagonal();
	MatrixXd Wn = MatrixXd::Identity(idx.size(),idx.size());

	calcForwardKinematics(CC);
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
		}
		else if( Ek2 < Ek ) Ek = Ek2;
		else
		{
			MoveJoints(idx, -dq);
			return true;
		}
	}
	return false;
}
