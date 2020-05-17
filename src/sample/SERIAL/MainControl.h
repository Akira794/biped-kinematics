#ifndef _MAINCONTROL_H_
#define _MAINCONTROL_H_
#include "Kinematics.h"
#include "cmd.h"
#include <cstdio>
#include <cstdlib>
//#include "../util/func.h"

//const double ready[6] = {};

class MainControl
{
public:
	MainControl(int argc, char *argv[], const double dt);
	~MainControl();
	void thread_run();
  void update_position();

	void SetDefaultAngle();
	void Now_State();
	void print_usage();
	void State();
	void PlotConf(int, int);
	void PlotArm();

	void position_list_clear(){
		target_list.clear();
	}
private:
	std::vector<Eigen::Vector4d> target_list;

	FILE *gp;

	int cmd, mode = 0;
	Eigen::Vector3d value, pos, att, now_pos;

  Link ulink[LINK_NUM];
	Kinematics *kinematics;
};

#endif
