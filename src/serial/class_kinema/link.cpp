#include "link.h"

extern "C" void SetJointInfo(struct Link *link)
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
