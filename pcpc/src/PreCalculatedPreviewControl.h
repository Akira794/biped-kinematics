#ifndef _PRECALCULATEDPREVIEWCONTROLLER_H_
#define _PRECALCULATEDPREVIEWCONTROLLER_H_

#include <stdio.h>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include<fstream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "FootStepPlanner.h"
#include "PlanCommon.h"
static const int NUM_COEF				= 15;
static const double WALKING_HALF_CYCLE	= 0.340;
static const double FOOT_WIDTH			= 0.050;
static const double SAMPLING_TIME		= 0.010;
static const double MAX_X_STRIDE			= 0.060;
static const double MAX_Y_STRIDE			= 0.060;
static const int MAX_DEG_STRIDE          = 9;


class PreCalculatedPreviewControl
{
public:
	std::vector<std::size_t> count_list;
	std::vector<Eigen::Vector2d> refzmp_list;
	std::vector<Eigen::Vector2d> com_list;
  std::vector<Eigen::Vector2d> vel_list;
  std::vector<Eigen::Vector2d> acc_list;
  std::vector<Eigen::Vector4d> target;
	std::vector<Eigen::Vector2d> route_list;

	std::vector<Eigen::Vector2d> p_com_list;
  std::vector<Eigen::Vector2d> p_vel_list;
  std::vector<Eigen::Vector2d> p_acc_list;

	std::vector<footstep_msgs::FootStatus> pre_support_leg_list;
	std::vector<footstep_msgs::WalkingStatus> pre_walking_status_list;
	std::vector<Eigen::Vector4d> pre_foot_step_list;
	std::vector<Eigen::Vector4d> pre_route_trajectory;
	std::vector<Eigen::Vector2d> pre_refzmp_list;

private:
	footstep_msgs::FootStatus next_leg_support;
	footstep_msgs::FootStatus ref_leg_sup;
	footstep_msgs::WalkingStatus walking_status;

public:
	FootStepPlanner *foot_planner;
public:
	PreCalculatedPreviewControl(const double _half_gait_cycle, const double _zmp_offset, const double _dt, footstep_msgs::FootStatus _sup_leg_state = footstep_msgs::LeftLegSup);
	~PreCalculatedPreviewControl();
  void list_clear(){
    count_list.clear();
    refzmp_list.clear();
    com_list.clear();
    vel_list.clear();
    acc_list.clear();

		p_com_list.clear();
    p_vel_list.clear();
    p_acc_list.clear();

		refzmp_list.push_back(Eigen::Vector2d::Zero());
    com_list.push_back(Eigen::Vector2d::Zero());
    vel_list.push_back(Eigen::Vector2d::Zero());
    acc_list.push_back(Eigen::Vector2d::Zero());

    p_com_list.push_back(Eigen::Vector2d::Zero());
    p_vel_list.push_back(Eigen::Vector2d::Zero());
    p_acc_list.push_back(Eigen::Vector2d::Zero());

		pre_support_leg_list.clear();
		pre_walking_status_list.clear();
		pre_foot_step_list.clear();
		pre_route_trajectory.clear();
/*
		pre_support_leg_list.push_back(footstep_msgs::BothLeg);
		pre_walking_status_list.push_back(footstep_msgs::StartWalking);
		pre_foot_step_list.push_back(Eigen::Vector4d::Zero());
		pre_route_trajectory.push_back(Eigen::Vector4d::Zero());
*/
  }
  void setTargetList(std::vector<Eigen::Vector4d> _target, footstep_msgs::WalkingStatus ref_walking_status = footstep_msgs::StartWalking){
    target = _target;
    num_target = target.size();
		walking_status = ref_walking_status;
  }

	void calcFootStepList(double );
  void setMaxStrideParameter(const double _max_stride_x, const double _max_stride_y, const double _max_stride_th){
		max_stride_x	= _max_stride_x;
		max_stride_y	= _max_stride_y;
		max_stride_th = _max_stride_th;
	}
  void mainLoop();
	void drawResult();
	void drawResultPCPCOnly();
	void setSupportLegModification();
	void interpolation_zmp_trajectory();
private:
	double dt, half_gait_cycle, zmp_offset;
  int step, walking;
  int num_target;
  double max_stride_x, max_stride_y, max_stride_th;
  int sign_deg;
  int flag_x;

	std::vector<footstep_msgs::FootStatus> support_leg_list;
	std::vector<footstep_msgs::WalkingStatus> walking_status_list;
	std::vector<Eigen::Vector4d> foot_step_list;
	std::vector<Eigen::Vector4d> route_trajectory;

public:
	std::vector<Eigen::Vector2d>getReferenceCom(){return  com_list; }
	std::vector<Eigen::Vector2d>getReferenceZMP(){return  refzmp_list; }
	std::vector<Eigen::Vector2d>getReferenceRoute(){return  route_list; }
	std::vector<std::size_t>getReferenceCountlist(){return count_list; }

	std::vector<footstep_msgs::FootStatus>getPreSupportLeg(){ return pre_support_leg_list; }
	std::vector<Eigen::Vector4d>getPreFootStepList(){ return pre_foot_step_list; }
	std::vector<Eigen::Vector4d>getPreRouteStepList(){ return pre_route_trajectory; }
	std::vector<Eigen::Vector2d>getPreRefZMPList(){ return pre_refzmp_list; }
};

#endif
