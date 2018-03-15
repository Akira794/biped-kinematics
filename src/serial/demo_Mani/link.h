#ifndef __LINK_H__
#define __LINK_H__

#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "demoManipLink.h"
#include "pi.h"

using namespace std;
using namespace Eigen;

struct Link
{
	string joint_name;					
	int parent;
	int child;
	int sister;						
	
	Matrix<double,3,1> p;
	Matrix<double,3,3> R;
	Matrix<double,3,1> a;
	Matrix<double,3,1> b;
	double q;
	double joint_mass;
	Link() : p(Matrix<double,3,1>::Zero()), R(Matrix<double,3,3>::Identity()),
					 a(Matrix<double,3,1>::Zero()), b(Matrix<double,3,1>::Zero()),
					 q(0.0)
	{
	}
};	
extern "C" void SetJointInfo( struct Link *link );

#endif
