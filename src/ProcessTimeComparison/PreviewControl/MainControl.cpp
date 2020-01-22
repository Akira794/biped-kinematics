#include "MainControl.h"

double rad2deg(double radian){
    return radian * 180/M_PI;
}

double deg2rad(int deg){
    return deg * M_PI / 180;
}

MainControl::MainControl(int argc, char *argv[], const double dt)
	: right_foot_position(Eigen::Vector3d::Zero()),
		left_foot_position(Eigen::Vector3d::Zero()),
    right_foot_position_base(Eigen::Vector3d(0,foot_dist_y,0)),
  	left_foot_position_base(Eigen::Vector3d(0,-foot_dist_y,0))
{
  position_list_clear();
	gait_generator	= new GaitPatternGenerator(0.34, 0.05, 0.01);
  pcpc = new PreCalculatedPreviewControl(0.34, 0.05, 0.01);

	gait_generator->setTrajectoryParameter(gait_cycle, foot_height_z, zmp_offset, right_foot_position_base, left_foot_position_base);
  /*
  target_x   = atof(argv[1]);
	target_y   = atof(argv[2]);
	target_deg = atof(argv[3]);
  */
  if (argc < 5 || (argc-1)%4 != 0){
		std::cerr << "Generate walking patterns by the Pre-Calculated Preview Control" << std::endl <<
		"Usage: CheckCoefficient <time> <x> <y> <th>[<time> <x> <y> <th>...]" << std::endl <<
        "EX. : 0.0 0.3 0.0 0  1.02 0.3 0.0 45 " << std::endl;
		exit(-1);
	}

	int num_target = (argc-1)/4;
	for(int i = 0; i < num_target; i ++){
    target.push_back(Eigen::Vector4d(atof(argv[i * 4 + 1]), atof(argv[i * 4 + 2]), atof(argv[i * 4 + 3]), atof(argv[i * 4 + 4]) ));
	}
  gait_generator->setTargetParameterList(target);
  //暫定
  target_x   = target[0](1);
  target_y   = target[0](2);
  target_deg = target[0](3);

}

MainControl::~MainControl()
{
}

void MainControl::thread_run()
{
    updateWalkingMotion();
    //DrawPCPC();
    //DrawResult();
}

void MainControl::updateWalkingMotion(){

  setControlParameter();
  //GenerateCOGTrajectory_PCPC();
  GenerateCOGTrajectory();
  GenerateFootTrajectory();

}
void MainControl::GenerateCOGTrajectory_PCPC(){
  gait_generator->pcpcsetup();
  gait_generator->loadPreCaluculatedFootList();
  setCountList(gait_generator->getPreCaluculatedReferenceCount());
  refzmp_list = gait_generator->getReferenceZMP();
  com_list = gait_generator->getReferenceCom();

}
void MainControl::setControlParameter(){
    gait_generator->ResetFootPlannerList();
    gait_generator->setFootDist(zmp_offset);
    gait_generator->setWalkPeriod(gait_cycle);
    gait_generator->setWalkFootHeight(foot_height_z);
    // Target Position
		// Foot Stride Parameter
		gait_generator->setFootMaxStrideParameter(MAX_X_STEP, MAX_Y_STEP, MAX_W_STEP);
		gait_generator->goTargetPos(target_x, target_y, deg2rad(target_deg));
}
void MainControl::GenerateCOGTrajectory(){
    boost::timer::cpu_timer process_timer;
    int count = 0;
    process_timer.start();
    while(1){
      if(!gait_generator->update()) break;
      com_list.push_back(gait_generator->getCenterOfMassPosition());
  		refzmp_list.push_back(gait_generator->get_preview_refzmp());
  		count_list.push_back(count++);
    }
    process_timer.stop();
    std::string timer_result = process_timer.format(9, "経過時間：%w秒\n" "ユーザーCPU処理時間：%u秒\n" "プロセスCPU処理時間：%s秒");
  	std::cout << timer_result << std::endl;
    gait_generator->loadFootplanner_step();
}
void MainControl::GenerateFootTrajectory(){
    //FootPlannerに基づく足先軌道生成
    gait_generator->generateReferenceFootTrajectory();
    left_foot_position_list  = gait_generator->getReferenceLeftFootPosition();
    right_foot_position_list = gait_generator->getReferenceRightFootPosition();
    right_foot_rot_list = gait_generator->getRightFootRot();
    left_foot_rot_list  = gait_generator->getLeftFootRot();

    //std::cout << "count, ZMP, COM, LF, RF, LROT, RROT,  " << count_list.size() << ", " << refzmp_list.size() << ", " << com_list.size() << ", " << left_foot_position_list.size() << ", " << right_foot_position_list.size() << ", " << right_foot_rot_list.size() << ", "<< right_foot_rot_list.size() <<", " << left_foot_rot_list.size() << std::endl;

    //移動座標系の足先軌道生成
    gait_generator->calcLocalFootposition();
    std::vector<Eigen::Vector2d> com_position;
    std::vector<Eigen::Vector3d> target_right_foot_pos, target_left_foot_pos;
    std::vector<double> right_foot_att, left_foot_att;

    com_position = gait_generator->getReferenceComPosition();
    target_left_foot_pos  = gait_generator->getReferenceTargetLeftFootPosition();
    target_right_foot_pos = gait_generator->getReferenceTargetRightFootPosition();
    right_foot_att = gait_generator->getTargetRightFootRot();
    left_foot_att  = gait_generator->getTargetLeftFootRot();
}

void MainControl::WalkingLoop(){

}
void MainControl::DrawPCPC(){
  FILE *gp = popen("gnuplot -persist\n", "w");
  gp = popen("gnuplot -persist\n", "w");
#if 1
  fprintf(gp, "set key left top\n");
  fprintf(gp, "set size ratio -1\n");
  fprintf(gp, "plot '-' with lines lw 2 lt 7 title \"com\" \
  , '-' with lines lw 2 lt 2 title \"refzmp\" \
  , '-' with points lw 1 title \"right foot pos\" \
  , '-' with points lw 1 title \"left foot pos\" \
  , '-' with vectors lc 'red' \
  , '-' with vectors lc 'green', \n");
  for(std::size_t i=0;i<right_foot_position_list.size();i++) fprintf(gp, "%f\t%f\n", com_list[i].x(), com_list[i].y()); fprintf(gp,"e\n");
  for(std::size_t i=0;i<right_foot_position_list.size();i++) fprintf(gp, "%f\t%f\n", refzmp_list[i](0), refzmp_list[i](1)); fprintf(gp,"e\n");
  for(std::size_t i=0;i<right_foot_position_list.size();i++) fprintf(gp, "%f\t%f\n", right_foot_position_list[i].x(), right_foot_position_list[i].y()); fprintf(gp,"e\n");
  for(std::size_t i=0;i<right_foot_position_list.size();i++) fprintf(gp, "%f\t%f\n", left_foot_position_list[i].x(), left_foot_position_list[i].y()); fprintf(gp,"e\n");
  for(std::size_t i=0;i<right_foot_position_list.size();i++) fprintf(gp, "%f\t%f\t%f\t%f\n", right_foot_position_list[i].x(), right_foot_position_list[i].y(), (0.01*cos(right_foot_rot_list[i])), (0.01*sin(right_foot_rot_list[i])) ); fprintf(gp,"e\n");
  for(std::size_t i=0;i<right_foot_position_list.size();i++) fprintf(gp, "%f\t%f\t%f\t%f\n", left_foot_position_list[i].x(), left_foot_position_list[i].y(), (0.01*cos(left_foot_rot_list[i])), (0.01*sin(left_foot_rot_list[i])) ); fprintf(gp,"e\n");
#endif
  fprintf(gp, "exit\n");
  pclose(gp);
}
void MainControl::DrawResult(){
  #if 1
  	FILE *gp = popen("gnuplot -persist\n", "w");
  #if 0
  	fprintf(gp, "set key left top\n");
  	fprintf(gp, "plot '-' with lines lw 2 title \"com\", '-' with lines lw 2 title \"refzmp\", '-' with lines lw 1 title \"right foot pos\", '-' with lines lw 1 title \"left foot pos\"\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], com_list[i].x()); fprintf(gp,"e\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], refzmp_list[i](0)); fprintf(gp,"e\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], right_foot_position_list[i].x()); fprintf(gp,"e\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], left_foot_position_list[i].x()); fprintf(gp,"e\n");
  #endif
  	fprintf(gp, "exit\n");
  	pclose(gp);

  	gp = popen("gnuplot -persist\n", "w");
  	#if 0
  	fprintf(gp, "set key left top\n");
  	//fprintf(gp, "set yrange[0.085:-0.085]\n");
  	fprintf(gp, "plot '-' with lines lw 2 title \"com\", '-' with lines lw 2 title \"refzmp\", '-' with lines lw 1 title \"right foot pos\", '-' with lines lw 1 title \"left foot pos\"\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], com_list[i].y()); fprintf(gp,"e\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], refzmp_list[i](1)); fprintf(gp,"e\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], right_foot_position_list[i].y()); fprintf(gp,"e\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], left_foot_position_list[i].y()); fprintf(gp,"e\n");
  	#endif
  	fprintf(gp, "exit\n");
  	pclose(gp);


  	gp = popen("gnuplot -persist\n", "w");
  #if 1
  	fprintf(gp, "set key left top\n");
  	fprintf(gp, "set size ratio -1\n");
  	fprintf(gp, "plot '-' with lines lw 2 lt 7 title \"com\" \
    , '-' with lines lw 2 lt 2 title \"refzmp\" \
    , '-' with points lw 1 title \"right foot pos\" \
    , '-' with points lw 1 title \"left foot pos\" \
    , '-' with vectors lc 'red' \
    , '-' with vectors lc 'green', \n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%f\t%f\n", com_list[i].x(), com_list[i].y()); fprintf(gp,"e\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%f\t%f\n", refzmp_list[i](0), refzmp_list[i](1)); fprintf(gp,"e\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%f\t%f\n", right_foot_position_list[i].x(), right_foot_position_list[i].y()); fprintf(gp,"e\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%f\t%f\n", left_foot_position_list[i].x(), left_foot_position_list[i].y()); fprintf(gp,"e\n");
    for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%f\t%f\t%f\t%f\n", right_foot_position_list[i].x(), right_foot_position_list[i].y(), (0.01*cos(right_foot_rot_list[i])), (0.01*sin(right_foot_rot_list[i])) ); fprintf(gp,"e\n");
    for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%f\t%f\t%f\t%f\n", left_foot_position_list[i].x(), left_foot_position_list[i].y(), (0.01*cos(left_foot_rot_list[i])), (0.01*sin(left_foot_rot_list[i])) ); fprintf(gp,"e\n");
  #endif
  #endif
  	fprintf(gp, "exit\n");
  	pclose(gp);

    #if 1
    gp = popen("gnuplot -persist\n", "w");
    fprintf(gp, "set terminal qt size 800,850 font \"Arial,8\" title 'demo_biped'\n");
    fprintf(gp, "set xrange [-0.2:0.35]\n");
    fprintf(gp, "set yrange [-0.2:0.35]\n");
    fprintf(gp, "set zrange [-0.02:0.35]\n");
    fprintf(gp, "set ticslevel 0\n");
    fprintf(gp, "set xtics 0.05\n");
    fprintf(gp, "set ytics 0.05\n");
    fprintf(gp, "set ztics 0.01\n");
    fprintf(gp, "set xlabel \"X(m)\"\n");
    fprintf(gp, "set ylabel \"Y(m)\"\n");
    fprintf(gp, "set zlabel \"Z(m)\"\n");
    fprintf(gp, "set size ratio -1\n");
    fprintf(gp, "set grid\n");
    fprintf(gp, "set view 60, 54\n");
    fprintf(gp, "set key left top\n");
    fprintf(gp, "set size ratio -1\n");
  	fprintf(gp, "splot '-' with lines lw 2 lt 7 title \"com\" \
    , '-' with lines lw 2 lt 2 title \"refzmp\" \
    , '-' with points lw 1 title \"right foot pos\" \
    , '-' with points lw 1 title \"left foot pos\",\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%f\t%f\t%f\n", com_list[i].x(), com_list[i].y(), 0.0f); fprintf(gp,"e\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%f\t%f\t%f\n", refzmp_list[i](0), refzmp_list[i](1), 0.0f); fprintf(gp,"e\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%f\t%f\t%f\n", right_foot_position_list[i].x(), right_foot_position_list[i].y(), right_foot_position_list[i].z()); fprintf(gp,"e\n");
  	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%f\t%f\t%f\n", left_foot_position_list[i].x(),  left_foot_position_list[i].y(),  left_foot_position_list[i].z() ); fprintf(gp,"e\n");

    #endif
  	fprintf(gp, "exit\n");
    pclose(gp);
  }
