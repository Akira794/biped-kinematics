#include <stdio.h>

#include "PreviewControl.h"
#include "FootStepPlanner.h"
#include "Kinematics.h"

const double dt = 0.01;
const double gait_cycle = 0.34;
const double foot_dist_y = 0.08;
const double foot_height_z = 0.05;
const double zmp_offset = 0.05;
const double HEIGHT_ZC  = 0.05;
const double ZMP_RANGE	= 0.060;
const double MAX_X_STEP	= 0.060;
const double MAX_Y_STEP	= 0.060;
const double MAX_W_STEP = 0.15708; //9deg 0.15708

double rad2deg(double radian){
    return radian * 180/M_PI;
}

double deg2rad(int deg){
    return deg * M_PI / 180;
}

int main(int argc, char *argv[]) {

	double target_x   = atof(argv[1]);
	double target_y   = atof(argv[2]);
	double target_deg = atof(argv[3]);

	int count = 0;

	Eigen::Vector3d right_foot_position(Eigen::Vector3d(0,foot_dist_y,0));
	Eigen::Vector3d left_foot_position(Eigen::Vector3d(0,-foot_dist_y,0));

  /* draw parameter*/
	std::vector<std::size_t> count_list;
	std::vector<Eigen::Vector2d> refzmp_list;
	std::vector<Eigen::Vector2d> com_list;
	std::vector<Eigen::Vector3d> right_foot_position_list, left_foot_position_list;
  std::vector<double> right_foot_rot_list, left_foot_rot_list;

  //歩行パターン生成開始
	GaitPatternGenerator gait_generator(gait_cycle, HEIGHT_ZC, dt);
	gait_generator.setFootMaxStrideParameter(MAX_X_STEP, MAX_Y_STEP, MAX_W_STEP);
	gait_generator.setTrajectoryParameter(gait_cycle, foot_height_z, zmp_offset, right_foot_position, left_foot_position);
	// 目標位置セット
	gait_generator.goTargetPos(target_x, target_y, deg2rad(target_deg));
	// 歩行パターン生成, ZMP軌道のアップデート
	while(1){
		// TODO 重心軌道、支持脚、遊脚軌道の更新(仮)
    if(!gait_generator.update()) break;
		com_list.push_back(gait_generator.getCenterOfMassPosition());
		refzmp_list.push_back(gait_generator.get_preview_refzmp());
		count_list.push_back(count++);

	}
  //足先軌道生成
  gait_generator.generateReferenceFootTrajectory();
  left_foot_position_list  = gait_generator.getReferenceLeftFootPosition();
  right_foot_position_list = gait_generator.getReferenceRightFootPosition();
  right_foot_rot_list = gait_generator.getRightFootRot();
  left_foot_rot_list  = gait_generator.getLeftFootRot();

  //移動座標系の足先軌道生成
  gait_generator.calcLocalFootposition();

  std::vector<Eigen::Vector3d> target_right_foot_pos, target_left_foot_pos;
  std::vector<double> right_foot_att, left_foot_att;

  target_left_foot_pos  = gait_generator.getReferenceTargetLeftFootPosition();
  target_right_foot_pos = gait_generator.getReferenceTargetRightFootPosition();
  right_foot_att = gait_generator.getTargetRightFootRot();
  left_foot_att  = gait_generator.getTargetLeftFootRot();





  #if 0
  	for(std::size_t i=0;i<foot_step.size();i++){

  		if(support_leg[i] == footstep_msgs::RightLegSup)
  			std::cout << "Right "<< "[" << i << "] " << "x: " << foot_step[i](1) << " " << "y: " << foot_step[i](2) << " " << "th: " << rad2deg(foot_step[i](3)) << std::endl;
  		else if(support_leg[i] == footstep_msgs::LeftLegSup)
  			std::cout << "Left " << "[" << i << "] " << "x: " << foot_step[i](1) << " " << "y: " << foot_step[i](2) << " " << "th: " << rad2deg(foot_step[i](3)) << std::endl;
  		else
  			std::cout << "Both " << "[" << i << "] " << "x: " << foot_step[i](1) << " " << "y: " << foot_step[i](2) << " " << "th: " << rad2deg(foot_step[i](3)) << std::endl;
  	}

  #endif


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
  //fprintf(gp, "set xrange [-0.2:0.35]\n");
  //fprintf(gp, "set yrange [-0.2:0.35]\n");
  //fprintf(gp, "set zrange [-0.02:0.1]\n");
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
