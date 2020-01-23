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
	kinematics		= new Kinematics(ulink);
  pcpc = new PreCalculatedPreviewControl(0.34, 0.05, 0.01);

	SetJointInfo(ulink);
	for(int i=0;i<JOINT_NUM;i++) kinematics->angle[i] = deg2rad(ready[i]);
	kinematics->setJointAngle();
	kinematics->calcForwardKinematics(CC);

//注意 左右逆  controllerでは相対視点から見て左右脚を決定してるため
	right_foot			          = ulink[LR2];
	left_foot					        = ulink[RR2];

  right_foot_position_init	= left_foot.p;
	left_foot_position_init		= right_foot.p;


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
    //DrawResult();
    WalkingLoop();
}

void MainControl::updateWalkingMotion(){

  setControlParameter();
  GenerateCOGTrajectory_PCPC();
  //GenerateCOGTrajectory();
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
    int count = 0;
    while(1){
      if(!gait_generator->update()) break;
      com_list.push_back(gait_generator->getCenterOfMassPosition());
  		refzmp_list.push_back(gait_generator->get_preview_refzmp());
  		count_list.push_back(count++);
    }
    gait_generator->loadFootplanner_step();
}
void MainControl::GenerateFootTrajectory(){
    //FootPlannerに基づく足先軌道生成
    gait_generator->generateReferenceFootTrajectory();
    left_foot_position_list  = gait_generator->getReferenceLeftFootPosition();
    right_foot_position_list = gait_generator->getReferenceRightFootPosition();
    right_foot_rot_list = gait_generator->getRightFootRot();
    left_foot_rot_list  = gait_generator->getLeftFootRot();
    com_att_list = gait_generator->getReferenceComAttitue();

    //std::cout << "count, ZMP, COM, LF, RF, LROT, RROT,  " << count_list.size() << ", " << refzmp_list.size() << ", " << com_list.size() << ", " << left_foot_position_list.size() << ", " << right_foot_position_list.size() << ", " << right_foot_rot_list.size() << ", "<< right_foot_rot_list.size() <<", " << left_foot_rot_list.size() << std::endl;

    //移動座標系の足先軌道生成
    gait_generator->calcLocalFootposition();

    /*com_list*/
    target_left_foot_pos  = gait_generator->getReferenceTargetLeftFootPosition();
    target_right_foot_pos = gait_generator->getReferenceTargetRightFootPosition();
    right_foot_att = gait_generator->getTargetRightFootRot();
    left_foot_att  = gait_generator->getTargetLeftFootRot();
}

void MainControl::LoopControl(){
  loop_cmd = 0;
  InputKey( &loop_cmd );
}

void MainControl::WalkingLoop(){
  dp = popen( "gnuplot -persist\n","w");
  SetPlotConf(dp, 60, 30);
  int index = 0;
  int com_deg = 0.0;
  bool flag = true;
  while(1){
    if(index+1 > com_list.size()){ flag = false; }
    LoopControl();
    if(loop_cmd == -1){ break;}
    //Now_State(0,0);
    usleep(10000);

    right_foot.p.x() = target_right_foot_pos[index].x() + left_foot_position_init.x();//左右逆に注意
    right_foot.p.y() = target_right_foot_pos[index].y() + left_foot_position_init.y();
    right_foot.p.z() = target_right_foot_pos[index].z() - 0.27;

    left_foot.p.x() = target_left_foot_pos[index].x() + right_foot_position_init.x();
    left_foot.p.y() = target_left_foot_pos[index].y() + right_foot_position_init.y();
    left_foot.p.z() = target_left_foot_pos[index].z() - 0.27;

    right_foot.R = kinematics->computeMatrixFromAngles(0, 0, right_foot_att[index]);//,right_foot_rot_list[index]);
    left_foot.R	 = kinematics->computeMatrixFromAngles(0, 0, left_foot_att[index]);//,left_foot_rot_list[index]);

    kinematics->calcInverseKinematicsAll(right_foot, left_foot);
    kinematics->setJointAngle();
    kinematics->calcForwardKinematics(CC);
    sendTargetAngle();
    if(flag){
      PlotLeg(dp, 0,0,0,0);
      index++;
    }
  }
  pclose(dp);
}

void MainControl::sendTargetAngle()
{
	std::vector<double> data(24,0);
  double out = 0.0;
  int sign = 1;
#if 1
	for(std::size_t index=0;index<data.size();index++){
    sign = 1;
    out = kinematics->angle[index];
    if( index == 1 || index == 2){
      sign = -1;
    }
	  std::cout << out*sign << "  ";
  }
  std::cout << std::endl;
#endif
}

void MainControl::Now_State(int mode, int foot )
{

	std::cout << ulink[RF].joint_name << " RF_pos = [" << ulink[RF].p(0) << ", " << ulink[RF].p(1) << ", "<< ulink[RF].p(2) << "]" << std::endl;
	std::cout << ulink[RF].joint_name << " RF_Rot = [" << rad2deg(ulink[RF].R(0)) << ", " << rad2deg(ulink[RF].R(1)) << ", "<< rad2deg(ulink[RF].R(2)) << "]" << std::endl;
  std::cout << std::endl;
  std::cout << ulink[LF].joint_name << " LF_pos = [" << ulink[LF].p(0) << ", " << ulink[LF].p(1) << ", "<< ulink[LF].p(2) << "]" << std::endl;
	std::cout << ulink[LF].joint_name << " LF_Rot = [" << rad2deg(ulink[LF].R(0)) << ", " << rad2deg(ulink[LF].R(1)) << ", "<< rad2deg(ulink[LF].R(2)) << "]" << std::endl;
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

void MainControl::SetPlotConf(FILE *gp, int roll, int yaw ){

	fprintf(gp, "set terminal qt size 800,850 font \"Arial,8\" title 'demo_biped'\n");
	fprintf(gp, "set xrange [-0.4:0.4]\n");
	fprintf(gp, "set yrange [-0.4:0.4]\n");
	fprintf(gp, "set zrange [-0.4:0.4]\n");
	fprintf(gp, "set ticslevel 0\n");
	fprintf(gp, "set xtics 0.05\n");
	fprintf(gp, "set ytics 0.05\n");
	fprintf(gp, "set ztics 0.05\n");
	fprintf(gp, "set xlabel \"X(m)\"\n");
	fprintf(gp, "set ylabel \"Y(m)\"\n");
	fprintf(gp, "set zlabel \"Z(m)\"\n");
	fprintf(gp, "set grid\n");
	fprintf(gp, "set view \"%d\",\"%d\", 1, 1.2 \n",roll,yaw );
}

double MainControl::_elm( int root, int elem ){
	return ulink[root].p(elem);
}

void MainControl::PlotLeg(FILE *gp, double c_x, double c_y, double c_z, double theta ){
	#if 1
	int TOTAL = LINK_NUM + 5;//5
	double x[TOTAL], y[TOTAL], z[TOTAL];
	double S = sin( theta );
	double C = cos( theta );

	int BCC = CC;
	int RFS = RY;
	int LFS = LY;
	int RFG = RF;
	int LFG = LF;

	int i = 0;

/* ===RF=== */

	x[BCC] = (_elm(BCC,0) + c_x);
	y[BCC] = (_elm(BCC,1) + c_y);
	z[BCC] = _elm(BCC,2);

	for( i = RFS; i <= RFG; i++ )
	{
    x[i] = (_elm(i,0) + c_x);
    y[i] = (_elm(i,1) + c_y);
    z[i] = _elm(i,2);
	}

	x[RFG+1] = (_elm(RFG,0) + 0.05);//0.05
	y[RFG+1] = (_elm(RFG,1) + c_y);
	z[RFG+1] = _elm(RFG,2);

/* ===LF=== */

	x[RFG+2] = (_elm(BCC,0) + c_x);
	y[RFG+2] = (_elm(BCC,1) + c_y);
	z[RFG+2] = _elm(BCC,2);

	for( i = LFS; i<= LFG; i++ )
	{
		x[i+2] = (_elm(i,0) + c_x);
		y[i+2] = (_elm(i,1) + c_y);
		z[i+2] = _elm(i,2);
	}

	x[LFG+3] = (_elm(LFG,0) + 0.05);//0.05
	y[LFG+3] = (_elm(LFG,1) + c_y);
	z[LFG+3] = _elm(LFG,2);

/* ==CC=== */

	x[LFG+4] = (_elm(BCC,0) + c_x);
	y[LFG+4] = (_elm(BCC,1) + c_y);
	z[LFG+4] = _elm(BCC,2);

	x[LFG+5] = (_elm(BCC,0) + c_x);
  y[LFG+5] = (_elm(BCC,1) + c_y);
  z[LFG+5] = _elm(BCC,2);

	fprintf(gp, "splot '-' with lines linetype 7 linewidth 5 title \"RLEG\",\
					'-' with lines linetype 6 linewidth 5 title \"LLEG\",\
					'-' with lines linetype 2 linewidth 5 title \"WAIST\",\n");

	for( i = BCC; i <= RFG+1; i++ )
	{
		fprintf( gp, "%f\t%f\t%f\n", x[i],y[i],z[i]);
	}

	fprintf(gp, "e\n");

	for( i = LFS+1; i <= LFG+3; i++ )
	{
		fprintf( gp, "%f\t%f\t%f\n", x[i],y[i],z[i]);
	}

	fprintf(gp, "e\n");

	for( i = LFG + 4; i < TOTAL; i++)
	{
		fprintf( gp, "%f\t%f\t%f\n", x[i],y[i],z[i]);
	}
	fprintf(gp, "e\n");

	fflush(gp);
	#endif
}
