#include "link.h"

void LinkInit(Link* link)
{
	for(int i = 0; i < JOINT_NUM; i++)
	{
		link[i].p = Vector3d::Zero();
		link[i].R = Matrix3d::Identity();
		link[i].a = Vector3d::Zero();
		link[i].b = Vector3d::Zero();
		link[i].q = 0.0;
		link[i].joint_mass = 0.0;
	}
}

void LinkDestroy(Link* link)
{
	LinkInit( link );
}


void _SetJointInfo(Link* link)
{
	for(int i = 0; i < JOINT_NUM; i++)
	{
		link[i].joint_name = joint_name[i];
		link[i].parent = parent[i];
		link[i].child = child[i];
		link[i].sister = sister[i];
		link[i].joint_mass = joint_mass[i];

		for(int j = 0; j < 3; j++)
		{
			link[i].a(j) = Axis[i][j];
			link[i].b(j) = Pos[i][j]/1000.0;
		}
	}
}

void _SetDefaultAngle(Link* link)
{
/*
	link[WAIST].q = Deg2Rad(0.0);
*/
	link[CC].q      = Deg2Rad(0.0);
	link[RLEG_J0].q = Deg2Rad(0.0);
	link[RLEG_J1].q = Deg2Rad(0.0);
	link[RLEG_J2].q = Deg2Rad(-10.0);
	link[RLEG_J3].q = Deg2Rad(20.0);
	link[RLEG_J4].q = Deg2Rad(-10.0);
	link[RLEG_J5].q = Deg2Rad(0.0);

	link[LLEG_J0].q = Deg2Rad(0.0);
	link[LLEG_J1].q = Deg2Rad(0.0);
	link[LLEG_J2].q = Deg2Rad(-10.0);
	link[LLEG_J3].q = Deg2Rad(20.0);
	link[LLEG_J4].q = Deg2Rad(-10.0);
	link[LLEG_J5].q = Deg2Rad(0.0);
}

void _OutputAngle(Link *link )
{
	cout << endl;
	for(int i = 0; i < JOINT_NUM; i++ )
	{
		cout << "angle[" << i << "] = " << Rad2Deg(link[i].q) << endl;
	}

}

void LinkDefaultInit( Link* link )
{
	_SetJointInfo( link );
	_SetDefaultAngle( link );	
}

void SetFootConf( Link* Target, double x, double y, double z, double roll, double pitch, double yaw)
{
	Target->p = Vector3d( x, y, z);
	Target->R = RotationFromRPY( Deg2Rad(roll), Deg2Rad(pitch), Deg2Rad(yaw));
 
}

double TotalMass( Link* link, int rootlink )
{
	double m = 0.0;
	
	if(rootlink == -1){
		m = 0.0;
	}else{
		m = link[rootlink].joint_mass + TotalMass( link, link[rootlink].sister ) + TotalMass( link, link[rootlink].child );

		return m;
	}
}
