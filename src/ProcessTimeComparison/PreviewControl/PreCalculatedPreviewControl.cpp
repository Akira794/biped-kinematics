#include "PreCalculatedPreviewControl.h"

PreCalculatedPreviewControl::PreCalculatedPreviewControl(const double _half_gait_cycle, const double _zmp_offset, const double _dt, footstep_msgs::FootStatus _sup_leg_state )
  : half_gait_cycle(_half_gait_cycle), dt(_dt), next_leg_support(_sup_leg_state), zmp_offset(_zmp_offset), step(0), walking(0), sign_deg(0), flag_x(0), num_target(0)
{
  list_clear();
  foot_planner = new FootStepPlanner(dt);
}

PreCalculatedPreviewControl::~PreCalculatedPreviewControl()
{
}

double __rad2deg(double radian){
    return radian * 180/M_PI;
}

double __deg2rad(int deg){
    return deg * M_PI / 180;
}

std::string to_string(float val, int digits){
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(digits) << val;
	return oss.str();
}

void PreCalculatedPreviewControl::setSupportLegModification(){
  Eigen::Vector2d end_com(Eigen::Vector2d(0,0));

  pre_support_leg_list.front() = footstep_msgs::BothLeg;
  pre_support_leg_list.back() = footstep_msgs::BothLeg;
  pre_support_leg_list.push_back(footstep_msgs::BothLeg);

  double com_count = pre_foot_step_list.back()(0);
  pre_foot_step_list.back()(0) +=   -WALKING_HALF_CYCLE + WALKING_HALF_CYCLE/2.0f;
  pre_route_trajectory.back()(0) += -WALKING_HALF_CYCLE + WALKING_HALF_CYCLE/2.0f;
  double size = pre_foot_step_list.back()(0) + 2.0f;
  pre_foot_step_list.push_back(Eigen::Vector4d( pre_foot_step_list.back()(0) + 2.0f ,pre_foot_step_list.back()(1),pre_foot_step_list.back()(2),pre_foot_step_list.back()(3)));
  pre_route_trajectory.push_back(Eigen::Vector4d( pre_route_trajectory.back()(0) + 2.0f,pre_route_trajectory.back()(1),pre_route_trajectory.back()(2),pre_route_trajectory.back()(3)));

  interpolation_zmp_trajectory();
}

void PreCalculatedPreviewControl::interpolation_zmp_trajectory(){
  size_t foot_step_count = 1;
	size_t size = pre_foot_step_list.size();
  double dt = SAMPLING_TIME;
	for(int t=0;t<(pre_foot_step_list[size-1](0)/dt);t++){
		double temp_time = t*dt;
		if(pre_foot_step_list[foot_step_count](0) < temp_time) foot_step_count++;
		pre_refzmp_list.push_back(Eigen::Vector2d(pre_foot_step_list[foot_step_count-1](1), pre_foot_step_list[foot_step_count-1](2)));
  }
}
void PreCalculatedPreviewControl::mainLoop(){

  int sampling_num_half_cycle = (half_gait_cycle + dt / 2) / dt;
  float current_x = 0.0f, current_y = 0.0f;
  float cog_x = 0.0f, cog_y = 0.0f;
	float vel_x = 0.0f, vel_y = 0.0f;
  float init_cog_x = 0.0f, init_cog_y = 0.0f;
  float init_vel_x = 0.0f, init_vel_y = 0.0f;

  //preview_control用
  float p_current_x = 0.0f, p_current_y = 0.0f;
  float p_cog_x = 0.0f, p_cog_y = 0.0f;
  float p_vel_x = 0.0f, p_vel_y = 0.0f;
  float p_init_cog_x = 0.0f, p_init_cog_y = 0.0f;
  float p_init_vel_x = 0.0f, p_init_vel_y = 0.0f;

	float ref_zmp_x = 0.0f, ref_zmp_y = 0.0f, ref_zmp_th = 0.0f;
  float prev_ref_zmp_x = 0.0f, prev_ref_zmp_y = 0.0f, prev_ref_zmp_th = 0.0f;
	float ref_zmp_x_gc = 0.0f, ref_zmp_y_gc = 0.0f, ref_zmp_th_gc = 0.0f;

  float trajectory_x = 0.0f, trajectory_y = 0.0f;
  float pos_trajectory_x = 0.0f, pos_trajectory_y = 0.0f;
  float length = 0.0f, stride_x = 0.0f, stride_y = 0.0f;
  float length_gc = 0.0f, stride_x_gc = 0.0f, stride_y_gc = 0.0f;

  float modi_x = 0.0f, modi_y = 0.0f;
  float modi_vel_x = 0.0f, modi_vel_y = 0.0f;
  float modi_acc_x = 0.0f, modi_acc_y = 0.0f;

  //preview_control用
  float p_modi_x = 0.0f, p_modi_y = 0.0f;
  float p_modi_vel_x = 0.0f, p_modi_vel_y = 0.0f;
  float p_modi_acc_x = 0.0f, p_modi_acc_y = 0.0f;

	float target_dx = 0.0f, target_dy = 0.0f, target_dth = 0.0f;
	float corr[NUM_COEF] = {0.0f};
  float precorr[NUM_COEF] = {0.0f};
	float foot_y = 0.0f;
	int end_time = 2000, last_step_start_time = 0;

  float step_dx = 0.0f, step_dy = 0.0f;
	float disp_x = 0.0f, disp_y = 0.0f;
	int dir = 1;
  int index = 0;

  float next_trajectory_x = 0.0f, next_trajectory_y = 0.0f, next_trajectory_th = 0.0f;
  int sign = 0;

  std::string com, prev_com;
	std::string com0 = "./GenerateWalkPattern ";
  FILE *fp;
  int support_foot = 0, step = 0, walking = 0;

  footstep_msgs::FootStatus support_leg = footstep_msgs::BothLeg;
  footstep_msgs::WalkingStatus walking_status = footstep_msgs::StartWalking;

  walking_status_list.push_back(walking_status);
  //ref_one_step_zmp.push_back(Eigen::Vector3d(0.0f, 0.0f, 0.0f));
  //ref_turn_point.push_back(Eigen::Vector3d(0.0f, 0.0f, 0.0f));
  foot_planner->SetFootStepParameter(max_stride_x, max_stride_y, max_stride_th, zmp_offset, half_gait_cycle);
  int count = 0;
  for(int i = 0; i < end_time; i ++){
// 現在時刻の計算
    float t = dt * i;
          // ロボットが歩行中
  		// 完全停止するフェーズの一個前
  		// 支持客切り替え時
  		// である場合は以下を実行
  	if ((i%sampling_num_half_cycle  == 0 && walking < 2) || (i%(sampling_num_half_cycle/2) == 0 && walking == 2)){
  		if (step < num_target - 1){
  			if (t > target[step + 1](0)){//time
            step ++;
            flag_x = 0;
        }
  		}

              // 以下、このコード内でGenerateWalkPattenを実行するための引数を計算

              // 旋回角を与えられた場合、の残り移動距離計算 //
              //std::cout << "sign  :" << sign << std::endl;
      int target_deg = 0, q = 0, sign = 0;
      int max_stride_deg = 9;
      if(target[step](3) != 0.0 && flag_x == 0){
        target_deg = fabs(target[step](3));
        while(target_deg>max_stride_deg){
          target_deg -= max_stride_deg;
            q++;
        }
        if(target_deg != 0) q+= 1;//9degで割れない場合は1ステップ追加する
                  //std::cout << q << std::endl;
        target[step](1) = ((target[step](1) < 0.0 )?(-1):(1))*max_stride_x*q;
        sign_deg = (target[step](3) > 0 ? 1 : -1 );
        flag_x = 1;
      }
      target[step](1) > 0.0 ? sign = 1 : sign = -1; // target[step].xが負のときマイナス符号を係数に掛ける
      if(target[step](3) == 0.0 && flag_x == 0) flag_x = 1;

              //旋回軌道の計算
      trajectory_x  = -(-foot_y)*sin(ref_zmp_th)+ref_zmp_x;//next tarjectory中心の座標系
      trajectory_y  =  (-foot_y)*cos(ref_zmp_th)+ref_zmp_y;
      length  = trajectory_x*cos(-ref_zmp_th) - (trajectory_y*sin(-ref_zmp_th)); //tarjectory移動量
              //std::cout << "steide_x, stride_y   :" << to_string(ref_zmp_x_gc,3)<<"," <<to_string(ref_zmp_y_gc,3) <<", " << to_string(rad2deg(ref_zmp_th_gc),2) << std::endl;

  		com = com0;
              // 歩行停止フェーズ出ない場合は以下を実行

  		if (walking < 2){
  			foot_y = FOOT_WIDTH * ((walking == 0) ? 0 : ((support_foot == 1) ? -1 : 1));

        if(target[step](3) == 0){//旋回角無しの場合

          target_dx = target[step](1) - ref_zmp_x_gc - ref_zmp_x;
          target_dy = target[step](2) - ref_zmp_y_gc - ref_zmp_y - foot_y;

        }else{//旋回角有りの場合
                      //std::cout << "steide_x, stride_y   :" << to_string(stride_x,3)<<"," <<to_string(stride_y,3) << std::endl;

          if(flag_x == 1){//stepが切り替わった最初の場合
            target_dx = target[step](1);// - length;
            length_gc = 0;
            stride_x_gc = target[step-1](1) - target_dx;
            stride_y_gc = target[step-1](2) - target_dy;
            flag_x = 2;
          }else{//step切り替わり以降
            target_dx = target[step](1) - length_gc - length;
            length_gc += length;
            stride_x_gc += stride_x;
            stride_y_gc += stride_y;
          }

          target_dy = target[step](2);
        }
        if(target[step](3) != 0){
          target_dth= __deg2rad(target[step](3))- ref_zmp_th_gc - ref_zmp_th;//現在の旋回角
        }else{
          target_dth = 0.0;
        }
                  //現在の重心位置の座標移動
        pos_trajectory_x  = - (foot_y)*sin( ref_zmp_th ) + ref_zmp_x;
        pos_trajectory_y  =   (foot_y)*cos( ref_zmp_th ) + ref_zmp_y;

        cog_x = current_x - pos_trajectory_x - ref_zmp_x_gc;
        cog_y = current_y - pos_trajectory_y - ref_zmp_y_gc;
        p_cog_x = p_current_x - pos_trajectory_x - ref_zmp_x_gc;
        p_cog_y = p_current_y - pos_trajectory_y - ref_zmp_y_gc;

      //現在の重心位置の回転座標
        init_cog_x   = (cog_x)*cos(-ref_zmp_th) - (cog_y)*sin(-ref_zmp_th);//next tarjectory中心の座標系
        init_cog_y   = (cog_x)*sin(-ref_zmp_th) + (cog_y)*cos(-ref_zmp_th);
        p_init_cog_x   = (p_cog_x)*cos(-ref_zmp_th) - (p_cog_y)*sin(-ref_zmp_th);//next tarjectory中心の座標系
        p_init_cog_y   = (p_cog_x)*sin(-ref_zmp_th) + (p_cog_y)*cos(-ref_zmp_th);

                  //現在の重心速度
        init_vel_x   = (vel_x)*cos(-ref_zmp_th)-(vel_y)*sin(-ref_zmp_th);
        init_vel_y   = (vel_x)*sin(-ref_zmp_th)+(vel_y)*cos(-ref_zmp_th);
        p_init_vel_x   = (p_vel_x)*cos(-ref_zmp_th)-(p_vel_y)*sin(-ref_zmp_th);
        p_init_vel_y   = (p_vel_x)*sin(-ref_zmp_th)+(p_vel_y)*cos(-ref_zmp_th);

  		} else foot_y = 0;
#if 0
              // GenerateWalkPatternの引数として追加
  		com += to_string(target_dx, 2) + " ";
  		com += to_string(target_dy, 2) + " ";
      com += to_string(__rad2deg(target_dth), 1) + " ";
  		com += to_string(p_init_vel_x, 4) + " ";
  		com += to_string(p_init_vel_y, 4) + " ";
  		com += to_string(p_init_cog_x, 4) + " ";
  		com += to_string(p_init_cog_y, 4) + " ";
  		com += std::to_string(support_foot) + " ";
  //            std::cout << "time, ref_zmp_th_gc, ref_zmp_th, prev_ref_zmp_th >= " \
              << t <<"," << ref_zmp_th_gc <<"," << ref_zmp_th << "," << prev_ref_zmp_th << std::endl;
#endif
              // 歩行フェーズが歩行中かつ旋回角有りかつ目標旋回角が0.01deg未満だった場合は以下を実行
      if (walking == 1 && (fabs(target[step](3))!=0) && fabs(__rad2deg(target_dth)) < 0.01f ){
  		    walking = 2;
  				end_time = i + sampling_num_half_cycle * 2;
  				last_step_start_time = i;
                  target[step](1) = 0.0;

              // 歩行フェーズが歩行中かつ旋回角有りかつ目標位置が0.01m未満だった場合は以下を実行
              }else if (walking == 1 && fabs(target_dx) < 0.01f && fabs(target_dy) < 0.01f){
                  walking = 2;
                  end_time = i + sampling_num_half_cycle * 2;
                  last_step_start_time = i;
                  target[step](1) = 0.0;
  			} else if (walking == 2) walking = 3;

  			com += std::to_string(walking) + " ";

        next_leg_support = (support_foot == 0) ? footstep_msgs::LeftLegSup : footstep_msgs::RightLegSup;
        switch(walking){
          case 0:
            walking_status = footstep_msgs::StartWalking;
            break;
          case 1:
            walking_status = footstep_msgs::Walking;
            break;

          case 2:
            walking_status = footstep_msgs::StopWalking;
            break;

          case 3:
            walking_status = footstep_msgs::Stop;
            break;
        }
        pre_walking_status_list.push_back(footstep_msgs::StartWalking);
  #if 0

  			std::cerr <<  com << " test.csv" << "   step: " << step << std::endl<< std::endl;
  #endif
      //com += "temp.csv";
              // GenerateWalkPatternの実行
#if 0
  			if ((fp = popen(com.c_str(), "r")) == NULL){
  				std::cerr <<  "error" << std::endl;
  				exit(-1);
  			}

              // GenerateWalkPatternで得られる多項式の係数を取得
  			for(int j = 0; j < NUM_COEF; j ++){
  				char str[256], *ptr;
  				fgets(str, 256, fp);
  				if (feof(fp)) break;
  				ptr=strchr(str, '\n');
  				if (ptr != NULL) *ptr = '\0';
  				//corr[j] = atof(str);
          precorr[j] = atof(str);

  //				printf("%s\n", str);
  			}
#endif
              // 以下がPre-Calculated Preview Control の実態部分
        foot_planner->SetFootStepParameter(max_stride_x, max_stride_y, max_stride_th, zmp_offset, half_gait_cycle); //これがないと何故かmax_stride_thがおかしな値になる
        foot_planner->SetTargetPos(target_dx, target_dy, target_dth, next_leg_support, walking_status);


  			// 上記でFootStepPlannerがGenerateWalkPattern内で実行されているため,PCPC単体で動かすには事前にFootStepPlannerの結果を入れておく必要がある
  			if (walking < 3){
                  disp_x = init_cog_x;
                  disp_y = init_cog_y;
                  dir = (support_foot == 1) ? -1 : 1;

  				prev_ref_zmp_x = ref_zmp_x;
  				prev_ref_zmp_y = ref_zmp_y;
                  prev_ref_zmp_th= ref_zmp_th;

          //foot_planner->SetFootStepParameter(max_stride_x, max_stride_y, max_stride_th, zmp_offset, half_gait_cycle); //これがないと何故かmax_stride_thがおかしな値になる
          //foot_planner->SetTargetPos(target_dx, target_dy, target_dth, next_leg_support, walking_status);

          ref_zmp_x  = foot_planner->foot_step_list[1](1);
          ref_zmp_y  = foot_planner->foot_step_list[1](2);
          ref_zmp_th = foot_planner->foot_step_list[1](3);

          //std::cout << "ref_zmp(x,y,th)   = " << t+0.34 <<" [s] , "<< ref_zmp_x << ", " << ref_zmp_y << ", " << ref_zmp_th << std::endl;

          ref_zmp_th_gc += prev_ref_zmp_th;//累積旋回角 9, 18, 27 ....
          ref_zmp_x_gc  += length*cos(ref_zmp_th_gc);
          ref_zmp_y_gc  += length*sin(ref_zmp_th_gc) + ((prev_ref_zmp_th != 0.0) ? 0.0f : (prev_ref_zmp_y + foot_y));//旋回しながら横移動は考えないものとする
          route_list.push_back(Eigen::Vector2d(ref_zmp_x_gc, ref_zmp_y_gc));

  				step_dx = (walking == 1) ? (-(-foot_y)*sin(ref_zmp_th)+ref_zmp_x): 0;
  				step_dy = (walking == 1) ? ((-foot_y)*cos(ref_zmp_th)+ref_zmp_y) : 0;
  			}
        //std::cout << "ref_zmp  :" << ref_zmp_x << ",  " << ref_zmp_y << ", " << std::endl;
  /*
              std::cout << rad2deg(target_dth) << std::endl;
              std::cout <<"dir   " << dir << std::endl;
              std::cout <<"init_vel   :" << init_vel_x << ", "<< init_vel_y << std::endl;
              std::cout <<" vel :  " << vel_x << ", " << vel_y << std::endl;

              std::cout << "rad:  " << std::atoi(to_string(rad2deg(target_dth), 1).c_str()) << std::endl;
  */
              if(target[step](3) != 0){
  #if 1
                  //std::cout << "target_dx  = " << target_dx<< std::endl;
                  if (walking == 0){
                      /*x_coef*/
                      corr[ 3] =   0.009832785   * init_vel_x +   1.0007521515 * disp_x + (sign_deg == 1 ? 0.000000961663 : 0.000001136605);
                      corr[ 4] =   0.9507822083  * init_vel_x +   0.0601223705 * disp_x + (sign_deg == 1 ? 0.00031604 : 0.000371252);
                      corr[ 5] =  -5.7900075     * init_vel_x -  17.9360475758 * disp_x + (sign_deg == 1 ? 0.0352379 : 0.0413779727);
                      corr[ 6] =  16.1962504167  * init_vel_x +  65.1242954545 * disp_x + (sign_deg == 1 ? - 0.0556214 : -0.0648901182);
                      corr[ 7] = -24.7802904167  * init_vel_x - 109.4457424242 * disp_x + (sign_deg == 1 ? 0.129835: 0.1502811818);
                      corr[ 8] =  16.8415858333  * init_vel_x +  77.4709848485 * disp_x + (sign_deg == 1 ? 0.0094961: 0.0135097727);

                      /*y_coef*/
                      corr[ 9] =   0.009832784   * init_vel_y +   1.0007519795 * disp_y + (sign_deg == 1 ? - 0.0000048335: -0.00000520858);
                      corr[10] =   0.9507828561  * init_vel_y +   0.0601223089 * disp_y + (sign_deg == 1 ? - 0.00143615: -0.00155966);
                      corr[11] =  -5.7900082121  * init_vel_y -  17.9360477273 * disp_y + (sign_deg == 1 ? - 0.166368: -0.180143);
                      corr[12] =  16.1962840909  * init_vel_y +  65.1242954545 * disp_y + (sign_deg == 1 ? 0.274715: 0.29629);
                      corr[13] = -24.7803065152  * init_vel_y - 109.4458333333 * disp_y + (sign_deg == 1 ? - 0.6510626: -0.702032);
                      corr[14] =  16.84158083939 * init_vel_y +  77.4709998485 * disp_y + (sign_deg == 1 ? 0.0173041: 0.0135777);

                  }else if (walking == 1 && fabs(__rad2deg(target_dth)) >18 ){
                      //std::cout << "walking =1 && target_deg > 27" << std::endl << std::endl;

                      corr[ 3] =  0.009832787  * init_vel_x + 1.0007500406  * disp_x + (sign_deg*dir == 1 ? 0.0000078982 : 0.0000092978);
                      corr[ 4] =  0.950783579  * init_vel_x + 0.0601222955  * disp_x + (sign_deg*dir == 1 ? 0.0024349103 : 0.0028486316);
                      corr[ 5] =  -5.7900075   * init_vel_x - 17.9360412121 * disp_x + (sign_deg*dir == 1 ? 0.2778302745 : 0.3259831687);
                      corr[ 6] =  16.1962795   * init_vel_x + 65.1241742424 * disp_x + (sign_deg*dir == 1 ? - 0.4503781273 : - 0.5306572909);
                      corr[ 7] =  -24.780304   * init_vel_x -109.4457045455 * disp_x + (sign_deg*dir == 1 ? 1.0567936818 : 1.2487176364);
                      corr[ 8] =  16.8415863   * init_vel_x + 77.4709624242 * disp_x + (sign_deg*dir == 1 ? 0.0169537527 : 0.0075202562);

                      /*y_coef*/
                      corr[ 9] =   0.0098327891 * init_vel_y +  1.0007523434 * disp_y +sign_deg*(sign_deg*dir == 1 ? 0.000050525333 :- 0.0000474444 );
                      corr[10] =   0.9507833182 * init_vel_y +  0.0601222929 * disp_y +sign_deg*(sign_deg*dir == 1 ? 0.0069223447   :- 0.0059604452 );
                      corr[11] =  -5.7899985758 * init_vel_y - 17.9360463636 * disp_y +sign_deg*(sign_deg*dir == 1 ? -0.44658046    :  0.5561602044 );
                      corr[12] =  16.1962519697 * init_vel_y + 65.1242383838 * disp_y +sign_deg*(sign_deg*dir == 1 ? 2.5193442667   :- 2.6965424578 );
                      corr[13] = -24.7803018182 * init_vel_y -109.4457979798 * disp_y +sign_deg*(sign_deg*dir == 1 ? -3.7339071333  :  4.1505080111 );
                      corr[14] =  16.8415836364 * init_vel_y + 77.4709777777 * disp_y +sign_deg*(sign_deg*dir == 1 ? 3.8625974667   : -3.8547755556 );

                  } else if (walking == 1 && fabs(__rad2deg(target_dth)) >9 && fabs(__rad2deg(target_dth)) <= 18){
                      //std::cout << "walking =1 && 9 < target_deg <= 18" << std::endl << std::endl;

                      /*x_coef xは完了 */
                      corr[ 3] =  0.009832794  * init_vel_x + 1.0007520335  * disp_x + (sign_deg*dir ==1 ? 0.0000077485   : 0.0000092019);
                      corr[ 4] =  0.950783158  * init_vel_x + 0.0601223258  * disp_x + (sign_deg*dir ==1 ? 0.0023901241   : 0.0028184398);
                      corr[ 5] =  -5.7900075   * init_vel_x - 17.9360412121 * disp_x + (sign_deg*dir ==1 ? 0.2728692745   : 0.3225767345);
                      corr[ 6] =  16.19628833  * init_vel_x + 65.124295454  * disp_x + (sign_deg*dir ==1 ? - 0.4432158727 : - 0.5252478);
                      corr[ 7] =  -24.7802904  * init_vel_x -109.4458272727 * disp_x + (sign_deg*dir ==1 ? 1.0399880545   : 1.2346576364);
                      corr[ 8] =  16.84158616  * init_vel_x + 77.4709618182 * disp_x + (sign_deg*dir ==1 ? 0.0134837927   : 0.0080239307);

                      /*y_coef*/
                      corr[ 9] =   0.0098327797 * init_vel_y +  1.0007501818 * disp_y + sign_deg*(sign_deg*dir ==1 ? 0.000050508    : - 0.0000475);
                      corr[10] =   0.9507829545 * init_vel_y +  0.060122303  * disp_y + sign_deg*(sign_deg*dir ==1 ? 0.006935942    : - 0.0060136377);
                      corr[11] =  -5.7900127727 * init_vel_y - 17.9360437374 * disp_y + sign_deg*(sign_deg*dir ==1 ? - 0.4451074933 : 0.5502668389);
                      corr[12] =  16.1962733939 * init_vel_y + 65.1242656566 * disp_y + sign_deg*(sign_deg*dir ==1 ? 2.5172636667   : - 2.68782107);
                      corr[13] = -24.78031      * init_vel_y -109.4458626263 * disp_y + sign_deg*(sign_deg*dir ==1 ? - 3.7305930667 : 4.1304679);
                      corr[14] =  16.8415742424 * init_vel_y + 77.4710333333 * disp_y + sign_deg*(sign_deg*dir ==1 ? 3.865175       : - 3.858574);


                  } else if (walking == 1 && fabs(__rad2deg(target_dth)) <= 9){
                      //std::cout << "walking =1 target_deg <= 9" << std::endl << std::endl;
                      int deg = std::atoi(to_string(__rad2deg(target_dth), 1).c_str());

                      /*x_coef xは */
                      corr[ 3] =  0.009832784  * init_vel_x + 1.0007520149  * disp_x +(dir == 1? 0.0000069324 - 0.00000008833*(deg-9) : 0.0000082461 + 0.00000006*(deg-9));
                      corr[ 4] =  0.950782083  * init_vel_x + 0.0601222879  * disp_x +(dir == 1? 0.0021187974 - 0.000025*(deg-9)      : 0.0025052356 + 0.000017883*(deg-9));
                      corr[ 5] =  -5.79000875  * init_vel_x - 17.9360493939 * disp_x +(dir == 1? 0.2426275418 - 0.0029096833*(deg-9)  : 0.2876681564 + 0.002093667*(deg-9));
                      corr[ 6] =  16.19628     * init_vel_x + 65.12425      * disp_x +(dir == 1? - 0.3953837791 + 0.0048103333*(deg-9): - 0.4700006655 - 0.0034781667*(deg-9));
                      corr[ 7] =  -24.7803091  * init_vel_x -109.445833333  * disp_x +(dir == 1? 0.9296123636 - 0.0114306667*(deg-9)  : 1.1072071636 + 0.0082956667*(deg-9));
                      corr[ 8] =  16.8415727   * init_vel_x + 77.4710143636 * disp_x +(dir == 1? 0.0045573247 + 0.000375*(deg-9)      : 0.0022232213 - 0.0003786667*(deg-9));

                      /*y_coef */
                      corr[ 9] =  0.009832795   * init_vel_y + 1.00075       * disp_y +(dir == 1? 0.0000493    + 0.00000012716*(deg-9) : - 0.0000469    + 0.00000014*(deg-9));
                      corr[10] =  0.950783      * init_vel_y + 0.06012225    * disp_y +(dir == 1? 0.0065439443 + 0.00003876*(deg-9)    : - 0.0058115708 + 0.00000425*(deg-9));
                      corr[11] =  -5.7900216667 * init_vel_y - 17.9360491667 * disp_y +(dir == 1? - 0.4887961811 + 0.0044458333*(deg-9): 0.5727868633 + 0.0048835667*(deg-9));
                      corr[12] =  16.196278275  * init_vel_y + 65.1243222222 * disp_y +(dir == 1? 2.5866410444 - 0.0072498333*(deg-9)  : - 2.72369832   - 0.0079741667*(deg-9));
                      corr[13] = -24.7803063333 * init_vel_y -109.4458305556 * disp_y +(dir == 1? - 3.8903879667 + 0.0170545*(deg-9)   : 4.2129882222 + 0.0187803333*(deg-9));
                      corr[14] =  16.8415775    * init_vel_y + 77.4710277778 * disp_y +(dir == 1? 3.852809     + 0.0000535*(deg-9)     : - 3.8524385222 - 0.00001245*(deg-9));

      			} else if ((walking == 2)&&((i - last_step_start_time) < sampling_num_half_cycle / 2)){ // 1st half
                      //std::cout << "walking =2 1st half" << std::endl << std::endl;
                      /*x_coef*/
                      corr[ 3] =  0.0098218192  * init_vel_x +  1.0007070909  * disp_x;
                      corr[ 4] =  0.9522823333  * init_vel_x +  0.065209703   * disp_x;
                      corr[ 5] = -5.8226        * init_vel_x -  18.002703030  * disp_x;
                      corr[ 6] = 16.3443616667  * init_vel_x +  64.45039394   * disp_x;
                      corr[ 7] = -24.0288625    * init_vel_x -  96.8950030303 * disp_x;
                      corr[ 8] = 12.2086808333  * init_vel_x +  34.6286030303 * disp_x;

                      /*y_coef */
                      corr[ 9] =  0.0098218188  * init_vel_y +  1.0007064444 * disp_y + 0.00005485466666 * dir;
                      corr[10] =  0.952283375   * init_vel_y +  0.0652096768 * disp_y + 0.0083833893 * dir;
                      corr[11] = -5.8226083333  * init_vel_y - 18.0027044444 * disp_y - 0.2648671667 * dir;
                      corr[12] = 16.24435       * init_vel_y + 64.4502343434 * disp_y + 2.0478333333 * dir;
                      corr[13] = -24.0288629167 * init_vel_y - 96.8951131313 * disp_y - 1.4731429333 * dir;
                      corr[14] = 12.208675      * init_vel_y + 34.628540404  * disp_y - 0.6107146667 * dir;

      			} else { // 2nd half
                      //std::cout << "walking =2 2nd half" << std::endl << std::endl;
                      /*x_coef*/
                      corr[ 3] = 0.0267749417  * init_vel_x + 1.0790043333  * disp_x;
                      corr[ 4] = 0.6303314167  * init_vel_x - 1.427         * disp_x;
                      corr[ 5] = -3.3560691667 * init_vel_x - 6.5826257576  * disp_x;
                      corr[ 6] = 6.8556975     * init_vel_x + 21.3300696    * disp_x;
                      corr[ 7] = -6.6099275    * init_vel_x - 23.8558333333 * disp_x;
                      corr[ 8] = 2.5156858333  * init_vel_x + 9.7315712121  * disp_x;
                      /*y_coef*/

                      corr[ 9] = 0.0267747083  * init_vel_y + 1.0790045455 * disp_y - 0.02272376   * dir;
                      corr[10] = 0.6303314583  * init_vel_y - 1.427        * disp_y + 0.282812     * dir;
                      corr[11] = -3.356070875  * init_vel_y - 6.5825979798 * disp_y - 1.0723469333 * dir;
                      corr[12] = 6.8557150833  * init_vel_y + 21.3299787879* disp_y + 1.9475796    * dir;
                      corr[13] = -6.6099213333 * init_vel_y -23.8557868687 * disp_y - 1.7747122667 * dir;
                      corr[14] = 2.5156853333  * init_vel_y +9.7315822222  * disp_y + 0.6545192533 * dir;

      			}
  #endif
  #if 0
                  std::cout << "Coefficient" << "  walk_state  :" << walking <<  std::endl;
                  for(int i = 3; i<15; i++){
                      std::cout << "No." << i-2 << "  = " << corr[i] << std::endl;
                  }
  #endif
              }else{
      /* default */
  #if 1
      			if (walking == 0){
      				corr[ 3] = 0.00986713 * vel_x +   1.00095 * disp_x + 0.00000150557;
      				corr[ 4] =   0.960351 * vel_x + 0.0784907 * disp_x + 9.39E-05;
      				corr[ 5] =   -5.88016 * vel_x -   18.2986 * disp_x + 0.0389385;
      				corr[ 6] =    16.6469 * vel_x +   67.6476 * disp_x - 0.0537971;
      				corr[ 7] =   -25.9567 * vel_x -   117.006 * disp_x + 0.109814;
      				corr[ 8] =    18.0493 * vel_x +   85.7187 * disp_x + 0.04934;
      				corr[ 9] = 0.00986713 * vel_y +   1.00095 * disp_y - 0.00000447877;
      				corr[10] =   0.960352 * vel_y + 0.0784909 * disp_y - 0.000996702;
      				corr[11] =   -5.88016 * vel_y -   18.2986 * disp_y - 0.172952;
      				corr[12] =    16.6469 * vel_y +   67.6476 * disp_y + 0.255042;
      				corr[13] =   -25.9567 * vel_y -   117.006 * disp_y - 0.541707;
      				corr[14] =    18.0493 * vel_y +   85.7184 * disp_y - 0.171364;
      			} else if (walking == 1 && fabs(target_dx) >= 0.18){
      				corr[ 3] = 0.00986713 * vel_x +   1.00095 * disp_x + 0.00000683605;
      				corr[ 4] =   0.960351 * vel_x + 0.0784907 * disp_x + 0.00210656;
      				corr[ 5] =   -5.88016 * vel_x -   18.2986 * disp_x + 0.308368;
      				corr[ 6] =    16.6469 * vel_x +   67.6476 * disp_x - 0.481904;
      				corr[ 7] =   -25.9567 * vel_x -   117.006 * disp_x + 1.07158;
      				corr[ 8] =    18.0493 * vel_x +   85.7187 * disp_x + 0.165453;
      				corr[ 9] = 0.00986713 * vel_y +   1.00095 * disp_y + 0.000113909 * step_dy + 0.0000566483 * dir;
      				corr[10] =   0.960352 * vel_y + 0.0784909 * disp_y +   0.0351079 * step_dy + 0.00659848   * dir;
      				corr[11] =   -5.88016 * vel_y -   18.2986 * disp_y +      5.1395 * step_dy - 0.517454     * dir;
      				corr[12] =    16.6469 * vel_y +   67.6476 * disp_y -     8.03195 * step_dy + 2.77058      * dir;
      				corr[13] =   -25.9567 * vel_y -   117.006 * disp_y +    17.86045 * step_dy - 4.50712      * dir;
      				corr[14] =    18.0493 * vel_y +   85.7184 * disp_y +     2.75671 * step_dy + 4.55405      * dir;
      			} else if (walking == 1 && fabs(target_dx) >= 0.12){
      				corr[ 3] =  0.00986713  * vel_x + 1.000954277 * disp_x + 1.27229E-05 * target_dx + 4.54573E-06;
      				corr[ 4] =  0.960350333 * vel_x + 0.078490216 * disp_x - 0.002620714 * target_dx + 0.002578286;
      				corr[ 5] = -5.880161404 * vel_x - 18.29858372 * disp_x + 0.066635714 * target_dx + 0.2963735  ;
      				corr[ 6] =  16.64685105 * vel_x + 67.64731602 * disp_x + 0.060257143 * target_dx - 0.492752143;
      				corr[ 7] = -25.95670404 * vel_x - 117.004961  * disp_x - 0.383785714 * target_dx + 1.140670714;
      				corr[ 8] =  18.04929175 * vel_x + 85.71714892 * disp_x + 0.784332143 * target_dx + 0.024265179;
      				corr[ 9] = 0.00986713 * vel_y +   1.00095 * disp_y + 0.000113909 * step_dy + 0.0000566483 * dir;
      				corr[10] =   0.960352 * vel_y + 0.0784909 * disp_y +   0.0351079 * step_dy + 0.00659848   * dir;
      				corr[11] =   -5.88016 * vel_y -   18.2986 * disp_y +      5.1395 * step_dy - 0.517454     * dir;
      				corr[12] =    16.6469 * vel_y +   67.6476 * disp_y -     8.03195 * step_dy + 2.77058      * dir;
      				corr[13] =   -25.9567 * vel_y -   117.006 * disp_y +    17.86045 * step_dy - 4.50712      * dir;
      				corr[14] =    18.0493 * vel_y +   85.7184 * disp_y +     2.75671 * step_dy + 4.55405      * dir;
      			} else if (walking == 1 && fabs(target_dx) >= 0.06){
      				corr[ 3] =  0.00986713  * vel_x + 1.000954277 * disp_x + 1.23591E-05 * target_dx + 4.58932E-06;
      				corr[ 4] =  0.960350333 * vel_x + 0.078490216 * disp_x + 0.004185036 * target_dx + 0.001761604;
      				corr[ 5] = -5.880161404 * vel_x - 18.29858372 * disp_x + 0.582346429 * target_dx + 0.234488107;
      				corr[ 6] =  16.64685105 * vel_x + 67.64731602 * disp_x - 0.956839286 * target_dx - 0.370698321;
      				corr[ 7] = -25.95670404 * vel_x - 117.004961  * disp_x + 2.213864286 * target_dx + 0.828943357;
      				corr[ 8] =  18.04929175 * vel_x + 85.71714892 * disp_x + 0.038196429 * target_dx + 0.113811464;
      				corr[ 9] =  0.00986713  * vel_y + 1.00095     * disp_y + 0.000050603 * target_dy + 0.0000555978 * dir;
      				corr[10] =  0.960352    * vel_y + 0.0784912   * disp_y +   0.0188648 * target_dy +   0.00693576 * dir;
      				corr[11] = -5.88016     * vel_y - 18.2986     * disp_y +     2.53641 * target_dy -     0.513661 * dir;
      				corr[12] =  16.6469     * vel_y + 67.6476     * disp_y -     4.04599 * target_dy +      2.74737 * dir;
      				corr[13] = -25.9567     * vel_y - 117.006     * disp_y +     9.12168 * target_dy -      4.42899 * dir;
      				corr[14] =  18.0493     * vel_y + 85.7184     * disp_y +    0.986705 * target_dy +      4.47628 * dir;
      			} else if (walking == 1){
      				corr[ 3] =  0.00986713  * vel_x + 1.000954277 * disp_x + 8.88479E-05 * target_dx - 2.2516E-22 ;
      				corr[ 4] =  0.960350333 * vel_x + 0.078490216 * disp_x + 0.033544948 * target_dx - 3.581E-20  ;
      				corr[ 5] = -5.880161404 * vel_x - 18.29858372 * disp_x + 4.490487582 * target_dx - 4.79369E-18;
      				corr[ 6] =  16.64685105 * vel_x + 67.64731602 * disp_x - 7.135199341 * target_dx + 7.61698E-18;
      				corr[ 7] = -25.95670404 * vel_x - 117.004961  * disp_x + 16.0297956  * target_dx - 3.18075E-20;
      				corr[ 8] =  18.04929175 * vel_x + 85.71714892 * disp_x + 1.934754945 * target_dx - 2.06539E-18;
      				corr[ 9] =  0.00986713  * vel_y + 1.00095     * disp_y + 0.000050603 * target_dy + 0.0000555978 * dir;
      				corr[10] =  0.960352    * vel_y + 0.0784912   * disp_y +   0.0188648 * target_dy +   0.00693576 * dir;
      				corr[11] = -5.88016     * vel_y - 18.2986     * disp_y +     2.53641 * target_dy -     0.513661 * dir;
      				corr[12] =  16.6469     * vel_y + 67.6476     * disp_y -     4.04599 * target_dy +      2.74737 * dir;
      				corr[13] = -25.9567     * vel_y - 117.006     * disp_y +     9.12168 * target_dy -      4.42899 * dir;
      				corr[14] =  18.0493     * vel_y + 85.7184     * disp_y +    0.986705 * target_dy +      4.47628 * dir;
      			} else if ((walking == 2)&&((i - last_step_start_time) < sampling_num_half_cycle / 2)){ // 1st half
      				corr[ 3] =  0.009842334 * vel_x + 1.000792528 * disp_x + 0.000343766 * step_dx;
      				corr[ 4] =  0.964866596 * vel_x + 0.110185519 * disp_x + 0.076905169 * step_dx;
      				corr[ 5] = -6.028117018 * vel_x - 19.41137766 * disp_x + 13.02902198 * step_dx;
      				corr[ 6] =  18.41986175 * vel_x + 82.09255844 * disp_x - 25.5853022  * step_dx;
      				corr[ 7] = -34.89211035 * vel_x - 196.8393784 * disp_x + 69.92306374 * step_dx;
      				corr[ 8] =  34.23064825 * vel_x + 246.0010589 * disp_x - 29.83588571 * step_dx;
      				corr[ 9] =  0.009842334 * vel_y + 1.000792528 * disp_y + 0.000343766 * step_dy + 5.68033E-05 * dir;
      				corr[10] =  0.964866596 * vel_y + 0.110185519 * disp_y + 0.076905169 * step_dy + 0.009354599 * dir;
      				corr[11] = -6.028117018 * vel_y - 19.41137766 * disp_y + 13.02902198 * step_dy - 0.319110648 * dir;
      				corr[12] =  18.41986175 * vel_y + 82.09255844 * disp_y - 25.5853022  * step_dy + 2.825179693 * dir;
      				corr[13] = -34.89211035 * vel_y - 196.8393784 * disp_y + 69.92306374 * step_dy - 6.344316758 * dir;
      				corr[14] =  34.23064825 * vel_y + 246.0010589 * disp_y - 29.83588571 * step_dy + 10.80424676 * dir;
      			} else { // 2nd half
      				corr[ 3] =  0.02698325  * vel_x + 1.079618597 * disp_x - 0.536945198 * step_dx;
      				corr[ 4] =  0.634442754 * vel_x - 1.432506511 * disp_x + 7.118750879 * step_dx;
      				corr[ 5] = -3.377784333 * vel_x - 6.561093506 * disp_x - 15.05317473 * step_dx;
      				corr[ 6] =  6.899324035 * vel_x + 21.28732727 * disp_x + 18.11159451 * step_dx;
      				corr[ 7] = -6.651343333 * vel_x - 23.81345498 * disp_x - 12.23722527 * step_dx;
      				corr[ 8] =  2.531252298 * vel_x + 9.71487368  * disp_x + 3.636932747 * step_dx;
      				corr[ 9] =  0.02698325  * vel_y + 1.079618597 * disp_y - 0.536945198 * step_dy - 0.022866289 * dir;
      				corr[10] =  0.634442754 * vel_y - 1.432506511 * disp_y + 7.118750879 * step_dy + 0.284312242 * dir;
      				corr[11] = -3.377784333 * vel_y - 6.561093506 * disp_y - 15.05317473 * step_dy - 1.080712981 * dir;
      				corr[12] =  6.899324035 * vel_y + 21.28732727 * disp_y + 18.11159451 * step_dy + 1.969947393 * dir;
      				corr[13] = -6.651343333 * vel_y - 23.81345498 * disp_y - 12.23722527 * step_dy - 1.802537787 * dir;
      				corr[14] =  2.531252298 * vel_y + 9.71487368  * disp_y + 3.636932747 * step_dy + 0.667591974 * dir;
      			}
              }
  #endif

              corr[3] += ref_zmp_x_gc;
              corr[9] += ref_zmp_y_gc;

              precorr[3] += ref_zmp_x_gc;
              precorr[9] += ref_zmp_y_gc;

              #if 0
                              std::cout << "Coefficient" << "  walk_state  :" << walking <<  std::endl;
                              for(int i = 3; i<15; i++){
                                  std::cout << "No." << i-2 << "  = " << corr[i] << std::endl;
                              }
              #endif

  			if (walking < 2){
  				vel_x = vel_y = 0.0f;
          p_vel_x = p_vel_y = 0.0f;
  				for(int j = 1; j < 6; j ++){
  					vel_x += j * corr[j+3] * std::pow(WALKING_HALF_CYCLE, j - 1);//WALKING_HALF_CYCLE
  					vel_y += j * corr[j+9] * std::pow(WALKING_HALF_CYCLE, j - 1);
            p_vel_x += j * precorr[j+3] * std::pow(WALKING_HALF_CYCLE, j - 1);//WALKING_HALF_CYCLE
  					p_vel_y += j * precorr[j+9] * std::pow(WALKING_HALF_CYCLE, j - 1);
  				}
                  //vel_x = (vel_x)*cos(-ref_zmp_th)-(vel_y)*sin(-ref_zmp_th);
                  //vel_y = (vel_x)*sin(-ref_zmp_th)+(vel_y)*cos(-ref_zmp_th);

  				if (walking == 1) support_foot ^= 1;
  			}
  		 	if (walking == 0) walking = 1;

  		}// 係数計算終了

          // 歩行開始から現在までの時間を計算
  		float dt = (walking < 2) ? (i % sampling_num_half_cycle) * SAMPLING_TIME : ((i - last_step_start_time) % (sampling_num_half_cycle * 2)) * SAMPLING_TIME;

          // 重心位置の計算
          current_x = current_y = 0.0f;
          p_current_x = p_current_y = 0.0f;
  		for(int j = 0; j < 6; j ++){
        current_x += corr[j+3] * std::pow(dt, j);
  			current_y += corr[j+9] * std::pow(dt, j);
        p_current_x += precorr[j+3] * std::pow(dt, j);
  			p_current_y += precorr[j+9] * std::pow(dt, j);
  		}

          //重心位置の軌道接続　旋回
          modi_x = (current_x-ref_zmp_x_gc)*cos(ref_zmp_th_gc) - (current_y-ref_zmp_y_gc)*sin(ref_zmp_th_gc)+ref_zmp_x_gc;//各歩毎でのtrajectory_pointを中心に旋回角分回転
          modi_y = (current_x-ref_zmp_x_gc)*sin(ref_zmp_th_gc) + (current_y-ref_zmp_y_gc)*cos(ref_zmp_th_gc)+ref_zmp_y_gc;//

          p_modi_x = (p_current_x-ref_zmp_x_gc)*cos(ref_zmp_th_gc) - (p_current_y-ref_zmp_y_gc)*sin(ref_zmp_th_gc)+ref_zmp_x_gc;//各歩毎でのtrajectory_pointを中心に旋回角分回転
          p_modi_y = (p_current_x-ref_zmp_x_gc)*sin(ref_zmp_th_gc) + (p_current_y-ref_zmp_y_gc)*cos(ref_zmp_th_gc)+ref_zmp_y_gc;//


  		// 重心速度の計算
  		double v_x = 0.0, v_y = 0.0;
      double p_v_x = 0.0, p_v_y = 0.0;
  		for(int j = 1; j < 6; j ++){
  			v_x += j * corr[j+3] * std::pow(dt, j - 1);
  			v_y += j * corr[j+9] * std::pow(dt, j - 1);
        p_v_x += j * precorr[j+3] * std::pow(dt, j - 1);
  			p_v_y += j * precorr[j+9] * std::pow(dt, j - 1);
  		}
          //重心速度の回転座標反映
          modi_vel_x = (v_x)*cos(ref_zmp_th_gc)-(v_y)*sin(ref_zmp_th_gc);
          modi_vel_y = (v_x)*sin(ref_zmp_th_gc)+(v_y)*cos(ref_zmp_th_gc);

          p_modi_vel_x = (p_v_x)*cos(ref_zmp_th_gc)-(p_v_y)*sin(ref_zmp_th_gc);
          p_modi_vel_y = (p_v_x)*sin(ref_zmp_th_gc)+(p_v_y)*cos(ref_zmp_th_gc);
          //std::cout << "modi]  " << prev_ref_zmp_th << std::endl;
          // 重心加速度の計算
  		double acc_x = 0.0, acc_y = 0.0;
      double p_acc_x = 0.0, p_acc_y = 0.0;
  		for(int j = 2; j < 6; j ++){
  			acc_x += j * (j - 1) * corr[j+3] * std::pow(dt, j - 2);
  			acc_y += j * (j - 1) * corr[j+9] * std::pow(dt, j - 2);
        p_acc_x += j * (j - 1) * precorr[j+3] * std::pow(dt, j - 2);
  			p_acc_y += j * (j - 1) * precorr[j+9] * std::pow(dt, j - 2);
  		}
          //重心速度の回転座標反映
          modi_acc_x = (acc_x)*cos(ref_zmp_th_gc)-(acc_y)*sin(ref_zmp_th_gc);
          modi_acc_y = (acc_x)*sin(ref_zmp_th_gc)+(acc_y)*cos(ref_zmp_th_gc);

          p_modi_acc_x = (p_acc_x)*cos(ref_zmp_th_gc)-(p_acc_y)*sin(ref_zmp_th_gc);
          p_modi_acc_y = (p_acc_x)*sin(ref_zmp_th_gc)+(p_acc_y)*cos(ref_zmp_th_gc);

  		if (walking == 2 && (i - last_step_start_time) == 34 / 2) foot_y = 0.0f;
      if(i%34 == 0){
        //pre_support_leg_list.push_back(footstep_msgs::BothLeg);
        //pre_walking_status_list.push_back(footstep_msgs::StartWalking);
        pre_foot_step_list.push_back(Eigen::Vector4d(t,ref_zmp_x_gc + fabs(foot_y)*cos(ref_zmp_th_gc+(-1)*((foot_y>0.00)-(foot_y<0.00))*(M_PI/2)), ref_zmp_y_gc + fabs(foot_y)*sin(ref_zmp_th_gc+(-1)*((foot_y>0.00)-(foot_y<0.00))*(M_PI/2)), ref_zmp_th_gc));
        pre_route_trajectory.push_back(Eigen::Vector4d(t,ref_zmp_x_gc,ref_zmp_y_gc,ref_zmp_th_gc));
        pre_support_leg_list.push_back(next_leg_support);
        /*
        std::cout << "time :" << t << ", " << ref_zmp_x_gc + fabs(foot_y)*cos(ref_zmp_th_gc+(-1)*((foot_y>0.00)-(foot_y<0.00))*(M_PI/2));\
        std::cout << ", " << ref_zmp_y_gc + fabs(foot_y)*sin(ref_zmp_th_gc+(-1)*((foot_y>0.00)-(foot_y<0.00))*(M_PI/2));
        std::cout << ", " << ref_zmp_th_gc << ",  ";
        std::cout << "route :" << ref_zmp_x_gc << ", " << ref_zmp_y_gc << ", " << ref_zmp_th_gc << std::endl;
        */
      }
      count_list.push_back(count++);
      refzmp_list.push_back(Eigen::Vector2d(   ref_zmp_x_gc + fabs(foot_y)*cos(ref_zmp_th_gc+(-1)*((foot_y>0.00)-(foot_y<0.00))*(M_PI/2)), ref_zmp_y_gc + fabs(foot_y)*sin(ref_zmp_th_gc+(-1)*((foot_y>0.00)-(foot_y<0.00))*(M_PI/2))));
      com_list.push_back(Eigen::Vector2d(modi_x, modi_y));
      vel_list.push_back(Eigen::Vector2d(modi_vel_x, modi_vel_y));
      acc_list.push_back(Eigen::Vector2d(modi_acc_x, modi_acc_y));

      p_com_list.push_back(Eigen::Vector2d(p_modi_x, p_modi_y));
      p_vel_list.push_back(Eigen::Vector2d(p_modi_vel_x, p_modi_vel_y));
      p_acc_list.push_back(Eigen::Vector2d(p_modi_acc_x, p_modi_acc_y));

  				//std::cout << SAMPLING_TIME * i << "," << ref_zmp_x_gc + fabs(foot_y)*cos(ref_zmp_th_gc+(-1)*((foot_y>0.00)-(foot_y<0.00))*(M_PI/2)) << "," \
  		        																	<< ref_zmp_y_gc + fabs(foot_y)*sin(ref_zmp_th_gc+(-1)*((foot_y>0.00)-(foot_y<0.00))*(M_PI/2)) << "," \
  																							<< modi_x << "," << modi_y << "," \
  		        																	<< modi_vel_x << "," << modi_vel_y << "," \
  																							<< modi_acc_x << "," << modi_acc_y << std::endl;

  	}
    setSupportLegModification();
}

void PreCalculatedPreviewControl::drawResult(){
    #if 1
    FILE *gp = popen("gnuplot -persist\n", "w");
    fprintf(gp, "set term qt 1 \n");
    fprintf(gp, "set title  \"Check Coefficient\"\n");
    fprintf(gp, "set xlabel \"x [m]\"\n");
    //fprintf(gp, "set xrange[-0.05:0.3]\n");
    fprintf(gp, "set ylabel \"y [m]\"\n");
    //fprintf(gp, "set yrange[-0.05:0.2]\n");
    fprintf(gp, "set size ratio -1\n");
    fprintf(gp, "set grid \n");
    fprintf(gp, "set key left top \n");
    fprintf(gp, "plot '-' with lines lw 3 lt 2 title \"RefZMP\" \
                     ,'-' with lines lw 3 lt 7 title \"COM\" \
                     ,'-' with points lw 2 lt 1 pt 7 title \"turn point\" \
                     ,'-' with lines lw 2 lt 6 title \"route trajectory\" \
                     ,'-' with lines lw 3 lt 9 dt 3 title \"PreviewControl COG\", \n");

    for(std::size_t i=0;i<refzmp_list.size();i++) fprintf(gp, "%f\t%f\n", refzmp_list[i].x(), refzmp_list[i].y()); fprintf(gp, "e\n");
    for(std::size_t i=0;i<com_list.size();i++) fprintf(gp, "%f\t%f\n", com_list[i].x(), com_list[i].y()); fprintf(gp,"e\n");

    for(std::size_t i=0;i<route_list.size();i++) fprintf(gp, "%f\t%f\n", route_list[i].x(), route_list[i].y()); fprintf(gp,"e\n");
    for(std::size_t i=0;i<route_list.size();i++) fprintf(gp, "%f\t%f\n", route_list[i].x(), route_list[i].y()); fprintf(gp,"e\n");
    for(std::size_t i=0;i<p_com_list.size();i++) fprintf(gp, "%f\t%f\n", p_com_list[i].x(), p_com_list[i].y()); fprintf(gp,"e\n");


    fprintf(gp,"exit\n");
    pclose(gp);
    #endif
}

void PreCalculatedPreviewControl::drawResultPCPCOnly(){
    #if 1
    FILE *gp = popen("gnuplot -persist\n", "w");
    fprintf(gp, "set term qt 1 \n");
    fprintf(gp, "set title  \"Check Coefficient\"\n");
    fprintf(gp, "set xlabel \"x [m]\"\n");
    //fprintf(gp, "set xrange[-0.05:0.3]\n");
    fprintf(gp, "set ylabel \"y [m]\"\n");
    //fprintf(gp, "set yrange[-0.05:0.2]\n");
    fprintf(gp, "set size ratio -1\n");
    fprintf(gp, "set grid \n");
    fprintf(gp, "set key left top \n");
    fprintf(gp, "plot '-' with lines lw 3 lt 2 title \"RefZMP\" \
                     ,'-' with lines lw 3 lt 7 title \"COM\" \
                     ,'-' with points lw 2 lt 1 pt 7 title \"turn point\" \
                     ,'-' with lines lw 2 lt 6 title \"route trajectory\" \n");

    for(std::size_t i=0;i<refzmp_list.size();i++) fprintf(gp, "%f\t%f\n", refzmp_list[i].x(), refzmp_list[i].y()); fprintf(gp, "e\n");
    for(std::size_t i=0;i<com_list.size();i++) fprintf(gp, "%f\t%f\n", com_list[i].x(), com_list[i].y()); fprintf(gp,"e\n");

    for(std::size_t i=0;i<route_list.size();i++) fprintf(gp, "%f\t%f\n", route_list[i].x(), route_list[i].y()); fprintf(gp,"e\n");
    for(std::size_t i=0;i<route_list.size();i++) fprintf(gp, "%f\t%f\n", route_list[i].x(), route_list[i].y()); fprintf(gp,"e\n");


    fprintf(gp,"exit\n");
    pclose(gp);
    #endif
}