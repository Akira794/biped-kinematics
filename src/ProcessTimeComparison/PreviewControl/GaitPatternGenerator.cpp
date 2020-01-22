#include "GaitPatternGenerator.h"

GaitPatternGenerator::GaitPatternGenerator(const double _half_gait_cycle, const double _height_z, const double _dt, footstep_msgs::FootStatus _sup_leg_state)
	: half_gait_cycle(_half_gait_cycle), com_pos_prev(Eigen::Vector2d::Zero()),
		height_z(_height_z), swing_trajectory_init(false), swing_step(0), gait_phase_step(0), dt(_dt),
		sup_leg_state(_sup_leg_state), ref_foot_pos_prev(Eigen::Vector2d::Zero()),
		right_foot_position(Eigen::Vector3d::Zero()), left_foot_position(Eigen::Vector3d::Zero()),
		ref_right_foot_position(Eigen::Vector3d::Zero()), ref_left_foot_position(Eigen::Vector3d::Zero()),
		right_foot_position_base(Eigen::Vector3d::Zero()), left_foot_position_base(Eigen::Vector3d::Zero()),
		right_foot_position_prev(Eigen::Vector3d::Zero()), left_foot_position_prev(Eigen::Vector3d::Zero()),
		p_goal(Eigen::Vector2d::Zero()), p_start(Eigen::Vector2d::Zero()), right_foot_rot(0), left_foot_rot(0),
		swing_trajectory(Eigen::Vector3d::Zero()), walking_status(footstep_msgs::StopWalking),
		side_step_move(false), planning_flag(false), walking_is_doing_flag(false), disable_calc_ref(false)
{
	preview_control = new PreviewControl(dt,1.6, 0.27);
	foot_planner = new FootStepPlanner(dt);
	pcpc = new PreCalculatedPreviewControl(half_gait_cycle, 0.05, dt);
	foot_position_list_clear();
	//pre_calculated_preview_control = new PreCalculatedPreviewControl();
}

GaitPatternGenerator::~GaitPatternGenerator()
{
}

void GaitPatternGenerator::loadFootplanner_step(){
	ResetFootPlannerList();
	setFootStepList(foot_planner->foot_step_list);
	setRouteStepList(foot_planner->route_trajectory);
	setSupLegList(foot_planner->support_leg_list);
}

void GaitPatternGenerator::loadPreCaluculatedFootList(){
	ResetFootPlannerList();
	setFootStepList(pcpc->getPreFootStepList());
	setRouteStepList(pcpc->getPreRouteStepList());
	setSupLegList(pcpc->getPreSupportLeg());
}

void GaitPatternGenerator::setPreCaluculatedReferenceCom(){
		com_step = pcpc->getReferenceCom();
}

void GaitPatternGenerator::pcpcsetup()
{
	pcpc->setTargetList(target_list);
	pcpc->setMaxStrideParameter(max_stride_x, max_stride_y, max_stride_th);
	pcpc->mainLoop();
	setPreCaluculatedReferenceCom();
	setPreCaluculatedReferenceCount();
	setPreCaluculatedReferenceZMP();
	//pcpc->drawResult();
	//pcpc->drawResultPCPCOnly();


	/*
	std::cout << "foot_step  :" << std::endl;
	for(int i = 0; i < foot_step.size(); i++){
		std::cout << foot_step[i](0) <<",  " <<  foot_step[i](1) <<",  " <<  foot_step[i](2) <<",  " <<  foot_step[i](3) << std::endl;
	}
	std::cout << "route_step  :" << std::endl;
	for(int i = 0; i < route_step.size(); i++){
		std::cout << route_step[i](0) <<",  " << route_step[i](1) <<",  " << route_step[i](2) <<",  " << route_step[i](3) << std::endl;
	}
	*/

}
void GaitPatternGenerator::setPreCaluculatedReferenceCount(){
	count_list =  pcpc->getReferenceCountlist();
}

void GaitPatternGenerator::setPreCaluculatedReferenceZMP(){
	refzmp_list = pcpc->getPreRefZMPList();
}
// TODO 両足支持期間かどうかの判定をFootStepPlanner側で管理
void GaitPatternGenerator::updateReferenceZmp()
{
	if(!planning_flag) return ;
	if(!walking_is_doing()){
		walking_status = footstep_msgs::StartWalking;
		if(0.0 <= dist_y) sup_leg_state = footstep_msgs::LeftLegSup;
		else sup_leg_state = footstep_msgs::RightLegSup;
		com_pos << 0.f, 0.f; com_vel << 0.f, 0.f; com_acc << 0.f, 0.f;
		com_pos_prev = com_pos;
		foot_planner->SetFootStepParameter(max_stride_x, max_stride_y, max_stride_th, zmp_offset, half_gait_cycle);
		foot_planner->SetTargetPos(dist_x, dist_y, dist_th, sup_leg_state, walking_status);
	}else if(walking_is_doing()){
		if(swing_trajectory_init) return ;
		if(calculation_walking_index%static_cast<int>(half_gait_cycle/dt) != 0) return ;
		if(calculation_walking_index%static_cast<int>(half_gait_cycle/dt) == 0) {
			int plan_index = calculation_walking_index / static_cast<int>(half_gait_cycle/dt);
			foot_planner->SetFootStepParameter(max_stride_x, max_stride_y, max_stride_th, zmp_offset, half_gait_cycle);
			foot_planner->SetCurrentPos(Eigen::Vector3d(foot_planner->foot_step_list[plan_index](1), foot_planner->foot_step_list[plan_index](2), foot_planner->foot_step_list[plan_index](3)));//add
			foot_planner->SetTargetPos(dist_x, dist_y, dist_th, foot_planner->support_leg_list[plan_index], footstep_msgs::Walking);
		}
	}
#if 0
	for(std::size_t i=0;i<foot_planner->foot_step_list.size();i++)
		std::cout << foot_planner->foot_step_list[i][0] << " " << foot_planner->foot_step_list[i][1] << " " << foot_planner->foot_step_list[i][2] << " " << foot_planner->foot_step_list[i][3] << std::endl;
	std::cout << std::endl;
#endif
	walking_is_doing_flag = true;
	gait_phase_step = 1;
	calculation_walking_index = 0;
	preview_control->interpolation_zmp_trajectory(foot_planner->foot_step_list);
	preview_control->set_com_param(com_pos, com_vel, com_acc);
	planning_flag = false; disable_calc_ref = false;
}

// 歩行軌道のアップデート
bool GaitPatternGenerator::update()
{
	updateReferenceZmp();
	//std::cout << "gait_phase_step  :" << gait_phase_step << std::endl;
	// 重心軌道(予見制御)のアップデート
	if(!preview_control->update(com_pos, com_vel, com_acc)) {
		walking_is_doing_flag = false;
		return false;
	}
	com_position_list.push_back(com_pos);

	// ZMPのアップデート(予見制御)
	preview_control->get_ref_zmp(temp_refzmp);
	preview_control->output_zmp(current_zmp);
#if 0
	FILE *fp = fopen("com_pos.csv", "a");
	fprintf(fp, "%f %f %f %f\n", temp_refzmp.x(), temp_refzmp.y(), com_pos.x(), com_pos.y());
	fclose(fp);
#endif
	// 足先軌道のアップデート
	//generateReferenceFootTrajectory();

	return true;
}
// 1ステップ分の遊脚軌道を計算
std::vector<Eigen::Vector3d> GaitPatternGenerator::calcSwingTrajectory()
{
  Eigen::Vector3d swing_pos(Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> swing_trajectory;

  for(int step=0;step<static_cast<int>(half_gait_cycle/dt);step++){
    double th = 2*M_PI/static_cast<int>((half_gait_cycle/dt)+0.001);
  	double temp = (th*step - sin(th*step)) / (2 * M_PI);

  	swing_pos.x() = p_start(0) + temp*(p_goal(0) - p_start(0));
  	swing_pos.y() = p_start(1) + temp*(p_goal(1) - p_start(1));
  	swing_pos.z() = height_z*0.5*(1-cos(th*step));
    swing_trajectory.push_back(Eigen::Vector3d(swing_pos(0), swing_pos(1), swing_pos(2)));
	}
		return swing_trajectory;
}
// FootStepPlannerの結果に基づく遊脚軌道生成　
void GaitPatternGenerator::generateReferenceFootTrajectory(){
	std::size_t index = 1;
  int swing_leg = 0;
  double dist_foot_rot = 0.0;
	double rot_prev = 0.0;
	double local_right_foot_rot = 0.0, local_left_foot_rot = 0.0;

	/*
	std::cout << "foot_step  :" << std::endl;
	for(int i = 0; i < foot_step.size(); i++){
		std::cout << foot_step[i](0) <<",  " <<  foot_step[i](1) <<",  " <<  foot_step[i](2) <<",  " <<  foot_step[i](3) << std::endl;
	}
	std::cout << "route_step  :" << std::endl;
	for(int i = 0; i < route_step.size(); i++){
		std::cout << route_step[i](0) <<",  " << route_step[i](1) <<",  " << route_step[i](2) <<",  " << route_step[i](3) << std::endl;
	}
	*/


//FootStepPlannerのリストを読み込む
	//初期脚配置の設定
	left_foot_position_prev  = left_foot_position_base;
	right_foot_position_prev = right_foot_position_base;
	double foot_dist_y = fabs(left_foot_position_base.y());
	//std::cout << "check foot_dist_y   " << foot_dist_y << std::endl;
	//index毎(foot_step_list分繰り返す)
	while(1){
		std::vector<Eigen::Vector3d> swing_trajectory;
    dist_foot_rot = route_step[index][3] - rot_prev; //旋回角の差分
      /* == Right === */
		if((sup_leg[index-1]) == footstep_msgs::RightLegSup){
      swing_leg = -1;
			if(index == ((foot_step.size())-1)){
				setTargetFootPos(Eigen::Vector2d(foot_step[index][1],left_foot_position_prev.y()), Eigen::Vector2d(left_foot_position_prev.x(),left_foot_position_prev.y()));
				swing_trajectory = calcSwingTrajectory();
			}else{
        ref_left_foot_position.x() = (route_step[index][1])+ foot_dist_y * cos(swing_leg*(M_PI/2)+(route_step[index][3]) );
        ref_left_foot_position.y() = (route_step[index][2])+ foot_dist_y * sin(swing_leg*(M_PI/2)+(route_step[index][3]) );
        ref_left_foot_position.z() = (route_step[index][3]);//旋回角

    		setTargetFootPos(Eigen::Vector2d(ref_left_foot_position.x(), ref_left_foot_position.y()), Eigen::Vector2d(left_foot_position_prev.x(), left_foot_position_prev.y()));
				swing_trajectory = calcSwingTrajectory();
			}
			for(int i=0;i<static_cast<int>(half_gait_cycle/dt);i++){
        right_foot_rot = -(dist_foot_rot/(static_cast<int>(half_gait_cycle/dt)))*(i+1);
        left_foot_rot  = (dist_foot_rot/(static_cast<int>(half_gait_cycle/dt)))*(i+1);
        if((sup_leg[index-2]) == footstep_msgs::BothLeg){left_foot_rot = 0.0;}
        if((sup_leg[index]) == footstep_msgs::BothLeg){
            //std::cout << "next_BothLeg" << std::endl;
            right_foot_rot = 0;
            left_foot_rot = -(local_left_foot_rot/(static_cast<int>(half_gait_cycle/dt)))*(i+1);
        }

        //std::cout << "Right_rot(R,L)  = " << rad2deg(right_foot_rot) << ",  " << rad2deg(left_foot_rot)<< std::endl;
        right_foot_rot_list.push_back(right_foot_rot);
        left_foot_rot_list.push_back(left_foot_rot);

				right_foot_position_list.push_back(right_foot_position_prev);
				left_foot_position_list.push_back(swing_trajectory[i]);
			}

      /* == Left ==*/
		}else if((sup_leg[index-1]) == footstep_msgs::LeftLegSup){
      swing_leg = 1;
			if(index == ((foot_step.size())-1)){
				setTargetFootPos(Eigen::Vector2d(foot_step[index][1],right_foot_position_prev.y()), Eigen::Vector2d(right_foot_position_prev.x(),right_foot_position_prev.y()));
				swing_trajectory = calcSwingTrajectory();
			}else{
        ref_right_foot_position.x() = (route_step[index][1])+ foot_dist_y * cos(swing_leg*(M_PI/2)+(route_step[index][3]));
        ref_right_foot_position.y() = (route_step[index][2])+ foot_dist_y * sin(swing_leg*(M_PI/2)+(route_step[index][3]));
        ref_right_foot_position.z() = (route_step[index][3]);//旋回角

        //std::cout << "ref_Rfoot_pos    = " << ref_right_foot_position.x() << ",  " << ref_right_foot_position.y() << ",  " << ref_right_foot_position.z() << std::endl;
				setTargetFootPos(Eigen::Vector2d(ref_right_foot_position.x(), ref_right_foot_position.y()), Eigen::Vector2d(right_foot_position_prev.x(), right_foot_position_prev.y()));
				swing_trajectory = calcSwingTrajectory();
			}
			for(int i=0;i<static_cast<int>(half_gait_cycle/dt);i++){
        right_foot_rot = (dist_foot_rot/(static_cast<int>(half_gait_cycle/dt)))*(i+1);
        left_foot_rot  = -(dist_foot_rot/(static_cast<int>(half_gait_cycle/dt)))*(i+1);
        if((sup_leg[index-2]) == footstep_msgs::BothLeg){right_foot_rot = 0.0;}
        if((sup_leg[index]) == footstep_msgs::BothLeg){
          //std::cout << "next_BothLeg" << std::endl;
          left_foot_rot = 0;
          right_foot_rot = -(local_right_foot_rot/(static_cast<int>(half_gait_cycle/dt)))*(i+1);
        }

        //std::cout << "Left_rot(R,L)  = " << rad2deg(right_foot_rot) << ",  " << rad2deg(left_foot_rot)<< std::endl;
        right_foot_rot_list.push_back(right_foot_rot);
        left_foot_rot_list.push_back(left_foot_rot);

				right_foot_position_list.push_back(swing_trajectory[i]);
				left_foot_position_list.push_back(left_foot_position_prev);
			}

		// TODO footstep_msgs::BothLegは両足支持期間をしめしているのではなくここでは歩き始めと停止を明示的に示している
		}else if(sup_leg[index-1] == footstep_msgs::BothLeg){
      right_foot_rot = 0.0;
      left_foot_rot = 0.0;
      //std::cout << "ref_Bothfoot_pos    = " << ref_right_foot_position.x() << ",  " << ref_right_foot_position.y() << ",  " << ref_right_foot_position.z() << std::endl;
			for(int i=0;i<static_cast<int>(half_gait_cycle/dt);i++){

        //std::cout << "Both_rot(R,L)  = " << rad2deg(right_foot_rot) << ",  " << rad2deg(left_foot_rot)<< std::endl;
        right_foot_rot_list.push_back(right_foot_rot);
        left_foot_rot_list.push_back(left_foot_rot);

				right_foot_position_list.push_back(right_foot_position_prev);
				left_foot_position_list.push_back(left_foot_position_prev);
			}
		}
    //startの更新
		right_foot_position_prev = right_foot_position_list.back();
		left_foot_position_prev = left_foot_position_list.back();

    local_left_foot_rot += left_foot_rot;
    local_right_foot_rot += right_foot_rot;
    //std::cout << "current_local : (R,L)   " << rad2deg(local_right_foot_rot) << ",  "<< rad2deg(local_left_foot_rot) << std::endl;

    rot_prev = route_step[index][3];
		index ++;
		if((foot_step.size()) < index) break;
	}

	for(int i=0;i<100;i++){
		right_foot_position_list.push_back(right_foot_position_prev);
		left_foot_position_list.push_back(left_foot_position_prev);
    right_foot_rot_list.push_back(right_foot_rot);
    left_foot_rot_list.push_back(left_foot_rot);

	}
}
void GaitPatternGenerator::calcLocalFootposition(){
	for(size_t i = 0; i < com_position_list.size(); i++){
		Eigen::Vector3d set_left_foot_position(Eigen::Vector3d::Zero());
		Eigen::Vector3d set_right_foot_position(Eigen::Vector3d::Zero());
		set_left_foot_position.x()  = right_foot_position_list[i](0) - com_position_list[i](0);
    set_left_foot_position.y()  = right_foot_position_list[i](1) - com_position_list[i](1);
    set_left_foot_position.z()  = right_foot_position_list[i](2);

    set_right_foot_position.x() = left_foot_position_list[i](0)  - com_position_list[i](0);
    set_right_foot_position.y() = left_foot_position_list[i](1)  - com_position_list[i](1);
    set_right_foot_position.z() = left_foot_position_list[i](2);

    target_right_foot_position_list.push_back(set_right_foot_position);
    target_left_foot_position_list.push_back(set_left_foot_position);
    target_right_foot_rot_list.push_back(right_foot_rot_list[i]);
    target_left_foot_rot_list.push_back(left_foot_rot_list[i]);
	}
}
#if 0
// 足先軌道の計算
void GaitPatternGenerator::generateReferenceFootTrajectory()
{
	right_foot_position.x() = right_foot_position_prev.x() - (com_pos.x() - com_pos_prev.x());
	right_foot_position.y() = right_foot_position_prev.y() - (com_pos.y() - com_pos_prev.y());
	right_foot_position.z() = 0.f;
	left_foot_position.x()  = left_foot_position_prev.x() - (com_pos.x() - com_pos_prev.x());
	left_foot_position.y()	= left_foot_position_prev.y() - (com_pos.y() - com_pos_prev.y());
	left_foot_position.z()	= 0.f;

	if(!swing_trajectory_init) generateSwingTrajectory();
	generateTrajectory();
	updateStatusParameter();

	ref_right_foot_position.x() = right_foot_position_base.x() + right_foot_position.x();
	ref_right_foot_position.y() = right_foot_position_base.y() + right_foot_position.y();
	ref_right_foot_position.z() = right_foot_position_base.z() + right_foot_position.z();

#ifdef GANKENKUN
	ref_left_foot_position.x() = left_foot_position_base.x() + left_foot_position.x();
#else
	ref_left_foot_position.x() = left_foot_position_base.x() - left_foot_position.x();
#endif
	ref_left_foot_position.y() = left_foot_position_base.y() + left_foot_position.y();
	ref_left_foot_position.z() = left_foot_position_base.z() + left_foot_position.z();

#if 1
	FILE *fp = fopen("foot_pos.csv", "a");
	fprintf(fp, "%f %f %f %f %f %f\n", ref_right_foot_position.x(), ref_right_foot_position.y(), ref_right_foot_position.z(), ref_left_foot_position.x(), ref_left_foot_position.y(), ref_left_foot_position.z());
	fclose(fp);
#endif
}
// パラメータのアップデート
void GaitPatternGenerator::updateStatusParameter()
{
	calculation_walking_index++;
	if(calculation_walking_index%static_cast<int>(half_gait_cycle/dt) == 0){
		swing_step = 0; swing_trajectory_init = false;
		gait_phase_step++;
	}
	if(walking_status == footstep_msgs::StartWalking){
		if(foot_planner->support_leg_list[gait_phase_step-1] != footstep_msgs::BothLeg)
			walking_status = footstep_msgs::Walking;
	}else if(walking_status == footstep_msgs::Walking){
		if(foot_planner->support_leg_list[gait_phase_step-1] == footstep_msgs::BothLeg)
			walking_status = footstep_msgs::StopWalking;
	}
	com_pos_prev = com_pos;
	right_foot_position_prev = right_foot_position;
	left_foot_position_prev = left_foot_position;
}

// 予見制御で得られた重心軌道に基づく足先軌道軌道生成
void GaitPatternGenerator::generateTrajectory()
{
	Eigen::Vector3d swing_trajectory = getSwingReferenceFootPosition();
	if(foot_planner->support_leg_list[gait_phase_step-1] == footstep_msgs::RightLegSup){
		left_foot_position = swing_trajectory;
		if(!side_step_move) left_foot_position.y() = left_foot_position_prev.y() - (com_pos.y() - com_pos_prev.y());
		//rotation
		right_foot_rot += (dist_foot_rot/(static_cast<int>(half_gait_cycle/dt)))*(swing_step-1);
		left_foot_rot  -= (dist_foot_rot/(static_cast<int>(half_gait_cycle/dt)))*(swing_step-1);
		//std::cout << "Right  => " << right_foot_rot* (180/M_PI) << "," <<left_foot_rot* (180/M_PI) <<  std::endl;

	}else if(foot_planner->support_leg_list[gait_phase_step-1] == footstep_msgs::LeftLegSup){
		right_foot_position = swing_trajectory;
		if(!side_step_move) right_foot_position.y() = right_foot_position_prev.y() - (com_pos.y() - com_pos_prev.y());
		//rotation
		right_foot_rot -= (dist_foot_rot/(static_cast<int>(half_gait_cycle/dt)))*(swing_step-1);
		left_foot_rot  += (dist_foot_rot/(static_cast<int>(half_gait_cycle/dt)))*(swing_step-1);
		//std::cout << "Left  => " << right_foot_rot* (180/M_PI) << "," <<left_foot_rot* (180/M_PI) <<  std::endl;
	}
}

// FootStepPlannerの結果に基づく遊脚軌道生成
// TODO 更新しなくていい時はしないようにする
void GaitPatternGenerator::generateSwingTrajectory()
{
	ref_foot_pos.x() = (foot_planner->foot_step_list[gait_phase_step](1) - foot_planner->foot_step_list[gait_phase_step-1](1))/2.0;
	ref_foot_pos.y() = (foot_planner->foot_step_list[gait_phase_step](2) - foot_planner->foot_step_list[gait_phase_step-1](2))/2.0;
	double dist_rot = foot_planner->foot_step_list[gait_phase_step](3) - foot_planner->foot_step_list[gait_phase_step-1](3);
	//std::cout << "step :" << gait_phase_step <<"," <<  ref_foot_pos.x() <<"," <<  ref_foot_pos.y() << "," << (dist_rot* (180/M_PI)) << std::endl;
	//std::cout << "y    "<< foot_planner->foot_step_list[gait_phase_step](2) <<",  "<<   foot_planner->foot_step_list[gait_phase_step-1](2) << std::endl;

	if(foot_planner->support_leg_list[gait_phase_step-1] == footstep_msgs::RightLegSup){
		if(gait_phase_step != (foot_planner->foot_step_list.size()-1))
			ref_foot_pos.y() += zmp_offset;
		if(gait_phase_step == (foot_planner->foot_step_list.size()-1))
			dist_rot = 0;
		setTargetFootPos(ref_foot_pos, Eigen::Vector2d(left_foot_position_prev.x(), left_foot_position_prev.y()));
		setTargetFootRot(dist_rot, right_foot_rot, left_foot_rot);
		//std::cout << "Left " << std::endl;
		//std::cout << "goal_pos = " << ref_foot_pos << ",   start_pos" << left_foot_position_prev.x() <<"," <<left_foot_position_prev.y()<< std::endl;

	}else if(foot_planner->support_leg_list[gait_phase_step-1] == footstep_msgs::LeftLegSup){
		if(gait_phase_step != (foot_planner->foot_step_list.size()-1))
			ref_foot_pos.y() -= zmp_offset;
		if(gait_phase_step == (foot_planner->foot_step_list.size()-1))
			dist_rot = 0;
		setTargetFootPos(ref_foot_pos, Eigen::Vector2d(right_foot_position_prev.x(), right_foot_position_prev.y()));
		setTargetFootRot(dist_rot, right_foot_rot, left_foot_rot);
		//std::cout << "Right" << std::endl;
		//std::cout << "goal_pos = " << ref_foot_pos << ",   start_pos" << right_foot_position_prev.x() <<"," <<right_foot_position_prev.y()<< std::endl;

	}
	//TODO 最後の歩幅の計算が上手くいって無さそう
	if(ref_foot_pos.y() == 0.0) side_step_move = false;
	else side_step_move = true;
	swing_trajectory_init = true;
}

// 時刻tにおける遊脚軌道
Eigen::Vector3d GaitPatternGenerator::getSwingReferenceFootPosition()
{
	double th = 2*M_PI/static_cast<int>((half_gait_cycle/dt)+0.001);
	double temp = (th*swing_step - sin(th*swing_step)) / (2 * M_PI);

	swing_trajectory.x() = p_start(0) + temp*(p_goal(0) - p_start(0));
	swing_trajectory.y() = p_start(1) + temp*(p_goal(1) - p_start(1));
	swing_trajectory.z() = height_z*0.5*(1-cos(th*swing_step));

	swing_step++;

	return swing_trajectory;
}
#endif
