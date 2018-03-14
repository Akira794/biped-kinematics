#include "robot.h"

void RobotInit( Robot *robot )
{
	LinkInit( robot->ulink);
	robot->com_pos = Vector3d::Zero();
	robot->rf_pos  = Vector3d::Zero();
	robot->lf_pos  = Vector3d::Zero();
	robot->cmd = 0;
	if( (robot->IK_mode) != 1) robot->IK_mode = 0;
	system( "stty -echo ");

	robot->fp = fopen( "angledata.csv", "w" );
	robot->sec_clock = 0.0;
}

void RobotPlotInit( Robot *robot, int roll, int yaw, int mode)
{
	if(mode == 0)
	{
		robot->gp = popen("gnuplot -persist","w");
	}
	else
	{
		robot->gp = popen("gnuplot ","w");
	}

	SetPlotConf( robot->gp, roll, yaw );
}

void RobotLoad( Robot *robot )
{
	LinkDefaultInit( robot->ulink );
	CalcForwardKinematics( robot->ulink, CC );

	robot->rf_pos[0] = 0.0;
	robot->rf_pos[1] = 0.0;
	robot->rf_pos[2] = - 0.25;

	robot->lf_pos[0] = 0.0;
	robot->lf_pos[1] = 0.0;
	robot->lf_pos[2] = - 0.25;
}

void RobotPoseInit( Robot *robot )
{
	RobotLoad( robot );
	RobotSetFootPos( robot );
	RobotSolveIK( robot );
}

void RobotPlotDestroy( Robot *robot )
{
	pclose(robot->gp);
}

void RobotDestroy( Robot *robot )
{
	LinkDestroy( robot->ulink );
	system( "stty echo" );

	fclose( robot->fp );

}

void _SetFootConf( Link* Target, Vector3d pos, int side )
{
	if( side == -1)
	{
		double ry = pos[1] -0.044;

		SetFootConf( Target, pos[0], ry,  pos[2],  0.0, 0.0, 0.0);
	}
	else if( side == 1)
	{
		double ly = pos[1] + 0.044;

		SetFootConf( Target, pos[0], ly,  pos[2],  0.0, 0.0, 0.0);
	}
}

void RobotSetFootPos( Robot *robot )
{
	_SetFootConf( &robot->Target_R, robot->rf_pos, -1);
	_SetFootConf( &robot->Target_L, robot->lf_pos,  1);

}

void RobotSolveIK( Robot *robot )
{
	clock_t start_clock, end_clock;
	start_clock = clock();
	if( (robot->IK_mode) ==0 )
	{
		InverseKinematicsAll( robot->ulink, robot->Target_R, robot->Target_L);
	}
	else if( (robot->IK_mode) ==1 )
	{
		InverseKinematics_LM_All( robot->ulink, robot->Target_R, robot->Target_L);
	}

	end_clock =clock();
	robot->sec_clock = (end_clock - start_clock)/(double)CLOCKS_PER_SEC;
}

void _NowPos( Robot *robot )
{
	cout << "    [" << robot->ulink[RLEG_J5].p[0] <<
	"," << robot->ulink[RLEG_J5].p[1] <<
	"," <<  robot->ulink[RLEG_J5].p[2] << "]" << endl;

}

void DrawLeg( Robot *robot )
{
	PlotLeg( robot->gp, robot->ulink, 0.0, 0.0, 0.0, -0.2 );
}

void _Now_IK_mode( Robot *robot )
{
	if( (robot->IK_mode) == 0)
	{
		cout << endl << "IK_mode: NR      " << endl;
	}
	else if( (robot->IK_mode ) == 1)
	{
		cout << endl << "IK_mode: LM      " << endl;
	}
}

void _Now_CostTime( Robot *robot )
{
	cout << "cost:" << (robot->sec_clock)*1000 << "msec" << endl;
}

void _NowState( Robot *robot )
{
	_NowPos( robot );
	_Now_IK_mode( robot );
	_Now_CostTime( robot );
	_OutputAngle( robot->ulink );
}

void _Output( Robot *robot )
{
	fprintf( robot->fp, "%lf,%lf,%lf,%lf\n",robot->ulink[RLEG_J5].p[0], 
	Rad2Deg(robot->ulink[RLEG_J2].q),
	Rad2Deg(robot->ulink[RLEG_J3].q),
	Rad2Deg(robot->ulink[RLEG_J4].q)
	);
}

void MoveFootPos( Robot *robot )
{
	if(kbhit())
	{
		Print_Usage();
		InputKey( &robot->cmd );
		if( (robot->cmd) == 1){
			(robot->rf_pos[1]) += 0.005;
			_NowState( robot );
		}
	
		else if( (robot->cmd) ==2){
			(robot->rf_pos[1]) -= 0.005;
			_NowState( robot );
		}
		
		else if( (robot->cmd) ==3){
			(robot->rf_pos[0]) -= 0.005;
			_NowState( robot );

			_Output( robot );
		}
	
		else if( (robot->cmd) ==4){
			(robot->rf_pos[0]) += 0.005;
			_NowState( robot );

			_Output( robot );
		}
	
		else if( (robot->cmd) ==5){
	    (robot->rf_pos[2]) += 0.005;
			_NowState( robot );
		}
	
		else if( (robot->cmd) ==6){
	    (robot->rf_pos[2]) -= 0.005;
			_NowState( robot );
		}

		else if( (robot->cmd) ==7){
			cout << "change NR_IK" << endl;
			robot->IK_mode = 0;
			_NowState( robot );
		}

		else if( (robot->cmd) ==8){
			cout << "change LM_IK" << endl;
			robot->IK_mode = 1;
			_NowState( robot );
		}

    else if( (robot->cmd) ==10){
			RobotInit( robot );
			RobotPoseInit( robot );
			_Now_IK_mode( robot );
			_OutputAngle( robot->ulink );
			
			_Output( robot );
		}

		else{
			cout << " " << endl;
		}
	}
}

void Print_Usage()
{
	cout << "=====keymap======" << endl;
	cout << "(X): 'e' or 'd'  " << endl;
	cout << "(Y): 's' or 'd'	" << endl;
	cout << "(Z): 'i' or 'k'	" << endl;
	cout << "NR : 'n'(default)" << endl;
	cout << "LM : 'l'					"	<< endl;
	cout << "reset: 'x'				" << endl;
	cout << "=================" << endl;
}
