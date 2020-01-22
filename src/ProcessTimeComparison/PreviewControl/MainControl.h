#ifndef _MAINCONTROL_H_
#define _MAINCONTROL_H_

#include "GaitPatternGenerator.h"
#include "Kinematics.h"
#include "PlanCommon.h"

#include <stdio.h>
#include <iostream>
#include <string>
#include <boost/timer/timer.hpp>

#ifdef GANKENKUN
const double ready[24] = {2,-15,15,0,-2,0,-2,15,-15,0,2,0,0,0,-16,-5,-40,0,16,5,40,0,0,0};
#else
const double ready[24] = {2,-15,15,2,-2,0,-2,15,-15,-2,2,0,0,0,-16,-5,-40,0,16,5,40,0,0,0};
#endif
const double dt = 0.01;
const double gait_cycle = 0.34;
const double foot_dist_y = 0.08;
const double foot_height_z = 0.05;
const double zmp_offset = 0.05;

const double MAX_X_STEP	= 0.060;
const double MAX_Y_STEP	= 0.060;
const double MAX_W_STEP = 0.15708; //9deg 0.15708

class MainControl
{
private:
  /* draw parameter*/
	std::vector<std::size_t> count_list;
	std::vector<Eigen::Vector2d> refzmp_list;
	std::vector<Eigen::Vector2d> com_list;
	std::vector<Eigen::Vector3d> right_foot_position_list, left_foot_position_list;
  std::vector<double> right_foot_rot_list, left_foot_rot_list;
	std::vector<Eigen::Vector4d> target;
public:
	MainControl(int argc, char *argv[], const double dt);
	~MainControl();
	void thread_run();;
	void updateWalkingMotion();
	void setControlParameter();
  void GenerateCOGTrajectory();
  void GenerateFootTrajectory();
	void GenerateCOGTrajectory_PCPC();
	void sendTargetAngle();
	void WalkingLoop();
  void DrawResult();
	void DrawPCPC();
  void position_list_clear(){
    count_list.clear();
    refzmp_list.clear();
    com_list.clear();
    right_foot_position_list.clear();
    left_foot_position_list.clear();
    right_foot_rot_list.clear();
    left_foot_rot_list.clear();
		target.clear();

  }
	void setCountList(std::vector<std::size_t>_count_list){count_list = _count_list; }
private:
  double target_x, target_y, target_deg;
	Eigen::Vector2d com_pos, ref_zmp;
	Eigen::Vector3d right_foot_position_base, left_foot_position_base;
	Eigen::Vector3d right_foot_position, left_foot_position;
  double right_foot_att, left_foot_att;
	Link right_foot, left_foot;

	Link ulink[LINK_NUM];
	GaitPatternGenerator *gait_generator;
	Kinematics *kinematics;
	PreCalculatedPreviewControl *pcpc;

};

#endif
