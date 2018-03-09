#include "robot.h"

#if 1
int main()
{
	Robot robot;
	RobotInit( &robot );
	RobotPlotInit( &robot, 54, 47, 1 );
	RobotLoad( &robot );

	while(1)
	{
		MoveFootPos( &robot );
		if( (robot.cmd) == -1)
		{
			break;
		}
		usleep(20000);

		RobotSetFootPos( &robot );
		RobotSolveIK( &robot );
		DrawLeg( &robot );
	}

	RobotPlotDestroy( &robot );
	RobotDestroy( &robot );

	return 0;
}
#endif
