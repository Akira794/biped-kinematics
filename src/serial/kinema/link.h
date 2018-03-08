#ifndef __LINK_H__
#define __LINK_H__

#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "demo_biped_link.h"
#include "pi.h"

using namespace std;
using namespace Eigen;

typedef struct
{
	string joint_name;					
	int parent;
	int child;
	int sister;						
					
	Vector3d p;	
	Matrix3d R;
	Vector3d a;	
	Vector3d b;	
	double q;
	double joint_mass;
}Link;	

void LinkInit(Link*);
void LinkDestroy(Link*);
void LinkDefaultInit(Link*);
void _OutputAngle(Link *link );
void SetFootConf(Link* , double, double, double , double , double, double);
double TotalMass( Link*, int );
#endif
