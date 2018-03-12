#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "link.h"
#include "kinematics.h"
#include "plot.h"
#include "cmd.h"
#include <cstdio>
#include <cstdlib>
#include <time.h>

using namespace std;
using namespace Eigen;

typedef struct
{
	Link ulink[JOINT_NUM], Target_R, Target_L;
	Vector3d com_pos, rf_pos, lf_pos;
	FILE *gp;
	int cmd, IK_mode;

	FILE *fp;
	double sec_clock;

}Robot;

void RobotInit( Robot *robot );
void RobotPlotInit(Robot *robot, int, int, int );
void RobotLoad( Robot *robot );
void RobotPoseInit( Robot *robot );
void RobotPlotDestroy( Robot *robot );
void RobotDestroy( Robot *robot );

void RobotSetFootPos( Robot *robot);
void RobotSolveIK( Robot *robot );
void MoveFootPos( Robot *robot );
void DrawLeg( Robot *robot );
void Print_Usage();
#endif
