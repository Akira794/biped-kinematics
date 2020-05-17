/*
 * @file		Kinematics.h
 * @brief		Calculation Kinematics
 * @author	Ryu Yamamoto
 * @date		2016/03/02
 */

#ifndef _KINEMATICS_
#define _KINEMATICS_

#include "Link.h"
#include "Jacobian.h"
#include <cmath>

namespace MotionControl
{
	class Kinematics
	{
	public:
		struct Link *ulink;
		double angle[JOINT_NUM];
	public:
		Kinematics(Link *ulink)
		{
			this->ulink = ulink;
		}
		void calcForwardKinematics(int rootlink);
		bool calcInverseKinematics(int to, Link target);
		bool calcLMInverseKinematics(int to, Link target);

		vector<int> FindRoute(int to);
		Matrix<double,3,3> Rodrigues(Matrix<double,3,1> a, double q);
		Matrix<double,3,1> rot2omega(Matrix<double,3,3> R);
		Eigen::Matrix<double,3,3> computeMatrixFromAngles(double roll, double pitch, double yaw);
		void computeAnglesFromMatrix(Eigen::Matrix<double,3,3> R, double &roll, double &pitch, double &yaw);
		Matrix<double,6,1> calcVWerr(Link Cref, Link Cnow);
		void MoveJoints( vector<int> idx, MatrixXd dq);
	};
};

#endif
