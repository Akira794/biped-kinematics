#ifndef _GAITPATTERNGENERATOR_H_
#define _GAITPATTERNGENERATOR_H_

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include "PlanCommon.h"
#include "PreviewControl.h"
#include "FootStepPlanner.h"
#include "PreCalculatedPreviewControl.h"


class GaitPatternGenerator{
private:
	Eigen::Vector3d right_foot_position, left_foot_position;
	Eigen::Vector3d ref_right_foot_position, ref_left_foot_position;
	Eigen::Vector3d right_foot_position_prev, left_foot_position_prev;
	Eigen::Vector3d right_foot_position_base, left_foot_position_base;
	Eigen::Vector2d com_pos_prev;
	Eigen::Vector2d ref_foot_pos, ref_foot_pos_prev;
	Eigen::Vector2d p_goal, p_start;

	//global_foot_position
	std::vector<Eigen::Vector4d> foot_step;
	std::vector<Eigen::Vector4d> route_step;
	std::vector<footstep_msgs::FootStatus>sup_leg;
	std::vector<Eigen::Vector2d> com_step;

	std::vector<Eigen::Vector3d> right_foot_position_list, left_foot_position_list;
	std::vector<double> right_foot_rot_list, left_foot_rot_list;
	//local foot position
	std::vector<Eigen::Vector2d> com_position_list;
	std::vector<double> com_att_list;
	std::vector<Eigen::Vector3d> target_right_foot_position_list, target_left_foot_position_list;
	std::vector<double> target_right_foot_rot_list, target_left_foot_rot_list;

	std::vector<Eigen::Vector4d> target_list;

public:
	PreviewControl *preview_control;
	FootStepPlanner *foot_planner;
	PreCalculatedPreviewControl *pcpc;

	std::vector<Eigen::Vector2d> refzmp_list;
	std::vector<std::size_t> count_list;

public:
	GaitPatternGenerator(const double _half_gait_cycle, const double _height_z, const double _dt, footstep_msgs::FootStatus _sup_leg_state=footstep_msgs::RightLegSup);
	~GaitPatternGenerator();
	void setTrajectoryParameter(double _half_gait_cycle, double _height_z, double _zmp_offset, Eigen::Vector3d _right_foot_position, Eigen::Vector3d _left_foot_position){
		swing_step = 0;
		half_gait_cycle = _half_gait_cycle;
		height_z = _height_z;
		zmp_offset = _zmp_offset;
		right_foot_position_base = _right_foot_position;
		left_foot_position_base = _left_foot_position;
		DoubleSupportRatio = 0.0;
		SingleSupportRatio = half_gait_cycle - DoubleSupportRatio;
	}

	void goTargetPos(const double target_x, const double target_y, const double target_th){
		dist_x	= target_x;
		dist_y	= target_y;
		dist_th = target_th;

		if(walking_status == footstep_msgs::StopWalking)
			walking_status = footstep_msgs::StartWalking;
		planning_flag = true;
	}
	void setFootMaxStrideParameter(const double _max_stride_x, const double _max_stride_y, const double _max_stride_th){
		max_stride_x	= _max_stride_x;
		max_stride_y	= _max_stride_y;
		max_stride_th = _max_stride_th;
	}
	void setTargetFootPos(Eigen::Vector2d _p_goal, Eigen::Vector2d _p_start){
		p_goal	= _p_goal;
		p_start = _p_start;
	}
	void setTargetFootRot(double _dist_foot_rot, double _right_foot_rot, double _left_foot_rot){
		dist_foot_rot		= _dist_foot_rot;
		right_foot_rot	= _right_foot_rot;
		left_foot_rot		= _left_foot_rot;
	}
	void setFootDist(double _zmp_offset){ zmp_offset = _zmp_offset; }
	void setWalkPeriod(double _half_gait_cycle){ half_gait_cycle = _half_gait_cycle; }
	void setWalkFootHeight(double _height_z){ height_z = _height_z; }

	void setFootStepList(std::vector<Eigen::Vector4d>_foot_list){ foot_step = _foot_list;}
	void setRouteStepList(std::vector<Eigen::Vector4d>_route_list){ route_step = _route_list;}
	void setSupLegList(std::vector<footstep_msgs::FootStatus>_sup_list){ sup_leg = _sup_list;}
	void setPreCaluculatedReferenceCom();
	void setPreCaluculatedReferenceCount();
	void setPreCaluculatedReferenceZMP();
	std::vector<std::size_t> getPreCaluculatedReferenceCount(){ return count_list; }
	std::vector<Eigen::Vector2d>getReferenceCom(){return com_step; }
	void ResetFootPlannerList(){
		foot_step.clear();
		route_step.clear();
		sup_leg.clear();
	}

	void loadFootplanner_step();
	void loadPreCaluculatedFootList();
	Eigen::Vector2d get_preview_refzmp(){ return temp_refzmp; }
	Eigen::RowVector2d get_preview_outzmp() { return current_zmp; }
	bool update();
	void updateReferenceZmp();
	#if 0
	void updateStatusParameter();
	void generateReferenceFootTrajectory();
	void generateTrajectory();
	void generateSwingTrajectory();
	Eigen::Vector3d	getSwingReferenceFootPosition();
	#endif
	void foot_position_list_clear(){
		right_foot_position_list.clear();
		left_foot_position_list.clear();
		right_foot_rot_list.clear();
		left_foot_rot_list.clear();

		com_att_list.clear();
		com_position_list.clear();
		target_right_foot_position_list.clear();
		target_left_foot_position_list.clear();
		target_right_foot_rot_list.clear();
		target_left_foot_rot_list.clear();
		target_list.clear();
	}
	//new method
	std::vector<Eigen::Vector3d> calcSwingTrajectory();
	void generateReferenceFootTrajectory();
	void calcLocalFootposition();
	void setTargetParameterList(std::vector<Eigen::Vector4d> _target){
    target_list = _target;
  }
	void pcpcsetup();

	std::vector<Eigen::Vector3d>getReferenceLeftFootPosition(){return  left_foot_position_list; }
	std::vector<Eigen::Vector3d>getReferenceRightFootPosition(){return right_foot_position_list; }
	std::vector<double>getRightFootRot(){return right_foot_rot_list;}
	std::vector<double>getLeftFootRot(){return left_foot_rot_list;}
	std::vector<Eigen::Vector3d>getReferenceTargetLeftFootPosition(){return  target_left_foot_position_list; }
	std::vector<Eigen::Vector3d>getReferenceTargetRightFootPosition(){return target_right_foot_position_list; }
	std::vector<double>getTargetRightFootRot(){return target_right_foot_rot_list;}
	std::vector<double>getTargetLeftFootRot(){ return target_left_foot_rot_list;}
	std::vector<Eigen::Vector2d>getReferenceComPosition(){return  com_position_list; }
	std::vector<Eigen::Vector2d>getReferenceZMP(){return refzmp_list; }
	std::vector<double>getReferenceComAttitue(){ return com_att_list; }
	/*
	Eigen::Vector3d getReferenceRightFootPosition(){ return ref_right_foot_position; }
	Eigen::Vector3d getReferenceLeftFootPosition(){ return ref_left_foot_position; }
	*/
	Eigen::Vector2d getCenterOfMassPosition(){ return com_pos; }
	footstep_msgs::WalkingStatus getWalkingStatus(){ return walking_status; }
	/*
	double getRightFootRot(){ return right_foot_rot; }
	double getLeftFootRot(){ return left_foot_rot; }
	*/
	bool walking_is_doing(){ return walking_is_doing_flag; }
private:
	double dt, half_gait_cycle;
	double dist_x, dist_y, dist_th;
	double max_stride_x, max_stride_y, max_stride_th;
	double dist_foot_rot;
	double right_foot_rot, left_foot_rot, com_rot;
	int swing_step, gait_phase_step;
	int SingleSupportRatio, DoubleSupportRatio;
	int calculation_walking_index;
	double height_z;
	double zmp_offset;
	bool swing_trajectory_init;
	bool side_step_move;
	bool walking_is_doing_flag;
	bool planning_flag;
	bool disable_calc_ref;
	Eigen::Vector2d com_pos, com_vel, com_acc;
	Eigen::Vector2d temp_refzmp;
	Eigen::RowVector2d current_zmp;
	Eigen::Vector3d swing_trajectory;
	footstep_msgs::WalkingStatus walking_status;
	footstep_msgs::FootStatus sup_leg_state;
};

#endif
