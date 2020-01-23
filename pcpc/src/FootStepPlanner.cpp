#include "FootStepPlanner.h"

FootStepPlanner::FootStepPlanner(const double _dt)
	: dt(_dt)
{
}

FootStepPlanner::~FootStepPlanner()
{
}

double _rad2deg(double radian){
    return radian * 180/3.14;
}

bool FootStepPlanner::SetTargetPos(const double target_x, const double target_y, const double target_th, footstep_msgs::FootStatus ref_leg_sup, footstep_msgs::WalkingStatus ref_walking_status)
{
	target_pos << target_x, target_y, target_th;
	next_leg_support = ref_leg_sup;
	walking_status = ref_walking_status;
	int th_foot_step_count = 0;
	foot_step_list_clear();

	target_distance = sqrt(pow(target_x, 2) + pow(target_y, 2));

	if(target_pos.x() == 0 && target_pos.y() == 0 && target_pos.z() == 0){
		stride.x() = stride.y() = stride.z() = 0.0;

	}else if(fabs(target_pos.x()) > fabs(target_pos.y())){
		foot_step_count = static_cast<int>((fabs(target_pos.x()) + stride.x() - 0.001) / stride.x());
		if(target_pos.x() < 0) stride.x() *= -1.0f;
		stride.y() = target_pos.y() / foot_step_count;

	}else{
		foot_step_count = static_cast<int>((fabs(target_pos.y()) + stride.y() - 0.001) / stride.y());
		if(target_pos.y() < 0) stride.y() *= -1.0f;
		stride.x() = target_pos.x() / foot_step_count;
	}
	//th用
	if(target_pos.z() != 0.0){
		th_foot_step_count = static_cast<int>((fabs(target_pos.z()) + stride.z() - 0.001) / stride.z());
	  if(target_pos.z() < 0.0) stride.z() *= -1.0f;

	  if(th_foot_step_count == foot_step_count){

	  }else if(th_foot_step_count < foot_step_count){
	            /*
	            cout << "####before:" << stride.th << endl;
	            stride.th = target_pos.th / foot_step_count;
	            cout << "####after:" << stride.th << endl;
	            */
	  	foot_step_count = th_foot_step_count;
	  }else{
	    foot_step_count = th_foot_step_count;
	  }

	}else{
	   stride.z() = 0.0;
	}


	bool result = target_pos_2_foot_step(ref_leg_sup);
	return result;

}
#if 1
bool FootStepPlanner::target_pos_2_foot_step(footstep_msgs::FootStatus ref_leg_sup)
{
	int counter = 0;

	start_walk(ref_leg_sup);
	Eigen::Vector3d turn_point(Eigen::Vector3d(0,0,0));

	next_control_point = {0.0f, 0.0f, stride.z()};
	    for(int i = 1; i < foot_step_count; i++){
	        time += step_time;
					support_leg_list.push_back(next_leg_support);
	        if(next_leg_support == footstep_msgs::RightLegSup){
	            next_leg_support = footstep_msgs::LeftLegSup;
	            leg_sign = 1;
	        }else if(next_leg_support == footstep_msgs::LeftLegSup){
	            next_leg_support = footstep_msgs::RightLegSup;
	            leg_sign = -1;
	        }
	        control_point = {
	            next_control_point.x() + stride.x()*cos(next_control_point.z()) + (fabs(zmp_offset))*cos(next_control_point.z() +leg_sign*(M_PI/2)),
	            next_control_point.y() + stride.x()*sin(next_control_point.z()) + (fabs(zmp_offset))*sin(next_control_point.z() +leg_sign*(M_PI/2))+ stride.y(),
	            next_control_point.z()
	        };
	//        cout  <<  time << "," << control_point.x << "," << control_point.y << "," << _rad2deg(control_point.th) << endl;
	        foot_step_list.push_back(Eigen::Vector4d(time, control_point.x(), control_point.y() , control_point.z()));

					turn_point.x() = zmp_offset*cos(-leg_sign*(M_PI/2)+control_point.z() );
					turn_point.y() = zmp_offset*sin(-leg_sign*(M_PI/2)+control_point.z() );
					turn_point.z() = control_point.z();
					//std::cout << "len_sign:  " << leg_sign << std::endl;

					route_trajectory.push_back(Eigen::Vector4d(time, turn_point.x()+control_point.x(), turn_point.y()+control_point.y() , turn_point.z()));
#if 1
					next_control_point = {
	                    next_control_point.x() + stride.x()*cos(next_control_point.z()),//次の原点
	                    next_control_point.y() + stride.x()*sin(next_control_point.z())+ stride.y(),
	                    next_control_point.z() + stride.z() //次の旋回角度
	        };
					walking_status_list.push_back(footstep_msgs::Walking);
	        //route_trajectory.push_back(Eigen::Vector4d(time, next_control_point.x(), next_control_point.y() , next_control_point.z()));
#endif
	    }
#if 0
	while(1){
		counter++;

		if(DistanceTargetPos(control_point) <= sqrt(pow(stride.x(),2)+pow(stride.y(),2))) break;
		if(foot_step_count < counter) return false;

		next_control_point = control_point + stride;
		time += step_time;
		support_leg_list.push_back(next_leg_support);

		if(next_leg_support == footstep_msgs::RightLegSup){
			foot_step_list.push_back(Eigen::Vector3d(time, next_control_point.x(), next_control_point.y()+zmp_offset));
			next_leg_support = footstep_msgs::LeftLegSup;
		}else if(next_leg_support == footstep_msgs::LeftLegSup){
			foot_step_list.push_back(Eigen::Vector3d(time, next_control_point.x(), next_control_point.y()-zmp_offset));
			next_leg_support = footstep_msgs::RightLegSup;
		}
		control_point = next_control_point;
	}
#endif
	stop_walk();
	#if 0
	    std::cout << std::endl << "total_foot_print" << std::endl;
	    std::size_t size = foot_step_list.size();
	    for(std::size_t i=0; i<size; i++){
	        std::cout << foot_step_list[i](0) <<","<<  foot_step_list[i](1) <<","<< foot_step_list[i](2) <<","<< (foot_step_list[i](3)) << std::endl;
	    }

			std::cout << std::endl << "route_list" << std::endl;
			size = route_trajectory.size();
			for(std::size_t i=0; i<size; i++){
				 std::cout<< route_trajectory[i](1) <<",   "<< route_trajectory[i](2) <<",   " << _rad2deg(route_trajectory[i](3)) << std::endl;
		 }

		 	std::cout << std::endl << "support_leg_list" << std::endl;
			size= support_leg_list.size();
			for(std::size_t i=0; i<size; i++){
				std::cout << support_leg_list[i] << std::endl;
			}
	#endif
	return true;
}

double FootStepPlanner::DistanceTargetPos(Eigen::Vector2d current_pos)
{
	return static_cast<double>(sqrt(pow((target_pos.x()-current_pos.x()),2)+pow((target_pos.y()-current_pos.y()),2)));
}

void FootStepPlanner::start_walk(footstep_msgs::FootStatus ref_leg_sup)
{
	if(walking_status == footstep_msgs::StartWalking){
		support_leg_list.push_back(footstep_msgs::BothLeg);
		walking_status_list.push_back(footstep_msgs::StartWalking);
		foot_step_list.push_back(Eigen::Vector4d(0.f, 0.f, 0.f, 0.f));
		route_trajectory.push_back(Eigen::Vector4d(0.f, 0.f, 0.f, 0.f));
		time += step_time;
	}

	double offset = 0.f;
	Eigen::Vector3d next_step;
	if(ref_leg_sup == footstep_msgs::RightLegSup){
		offset = zmp_offset;
		next_leg_support = footstep_msgs::LeftLegSup;
	}else if(ref_leg_sup == footstep_msgs::LeftLegSup){
		offset = -1 * zmp_offset;
		next_leg_support = footstep_msgs::RightLegSup;
	}
	if(walking_status == footstep_msgs::StartWalking)
		next_step << 0.f, offset, 0.0f;

	support_leg_list.push_back(ref_leg_sup);
	foot_step_list.push_back(Eigen::Vector4d(time, next_step.x(), next_step.y(), 0.0f));
	route_trajectory.push_back(Eigen::Vector4d(time, 0.0f, 0.0f, 0.0f));
	walking_status_list.push_back(footstep_msgs::Walking);
}

void FootStepPlanner::stop_walk()
{
	Eigen::Vector3d turn_point(Eigen::Vector3d(0,0,0));
	if(target_pos.z() != 0.0){
        next_control_point.z() = target_pos.z();
    }else if(target_pos.z() == 0.0){
        stride.x() = target_pos.x() - next_control_point.x();
    }

		if(walking_status != footstep_msgs::StopWalking){
			time += step_time;
			support_leg_list.push_back(next_leg_support);
		  if(next_leg_support == footstep_msgs::RightLegSup){
		  	leg_sign = 1;
		  }else if(next_leg_support == footstep_msgs::LeftLegSup){
		  	leg_sign = -1;
		  }
			control_point = {
		  	next_control_point.x() + stride.x()*cos(next_control_point.z()) + (fabs(zmp_offset))*cos(next_control_point.z() +leg_sign*(M_PI/2)),
		  	next_control_point.y() + stride.x()*sin(next_control_point.z()) + (fabs(zmp_offset))*sin(next_control_point.z() +leg_sign*(M_PI/2))+ stride.y(),
		  	next_control_point.z()
		  };
			next_control_point = {
		  	next_control_point.x() + stride.x()*cos(next_control_point.z()),//次の原点
		  	next_control_point.y() + stride.x()*sin(next_control_point.z())+ stride.y(),
		  	next_control_point.z() + 0
		  };

			turn_point.x() = zmp_offset*cos(-leg_sign*(M_PI/2)+control_point.z() );
			turn_point.y() = zmp_offset*sin(-leg_sign*(M_PI/2)+control_point.z() );
			turn_point.z() = control_point.z();

		 route_trajectory.push_back(Eigen::Vector4d(time, control_point.x()+turn_point.x(), control_point.y()+turn_point.y() , control_point.z()));
     foot_step_list.push_back(Eigen::Vector4d(time, control_point.x(), control_point.y(), control_point.z()));
		 walking_status_list.push_back(footstep_msgs::StopWalking);
		}
		time += step_time/2.0f;
		support_leg_list.push_back(footstep_msgs::BothLeg);
    route_trajectory.push_back(Eigen::Vector4d(time, next_control_point.x(), next_control_point.y() , next_control_point.z()));
		foot_step_list.push_back(Eigen::Vector4d(time, next_control_point.x(), next_control_point.y(), next_control_point.z()));
		walking_status_list.push_back(footstep_msgs::Stop);

		support_leg_list.push_back(footstep_msgs::BothLeg);
    route_trajectory.push_back(Eigen::Vector4d(time+2.0f, next_control_point.x(), next_control_point.y() , next_control_point.z()));
		foot_step_list.push_back(Eigen::Vector4d(time+2.0f, next_control_point.x(), next_control_point.y(), next_control_point.z()));
		walking_status_list.push_back(footstep_msgs::Stop);

	#if 0
	next_control_point = target_pos;

	if(walking_status != footstep_msgs::StopWalking){
		time += step_time;
		support_leg_list.push_back(next_leg_support);
		if(next_leg_support == footstep_msgs::RightLegSup)
			foot_step_list.push_back(Eigen::Vector3d(time, next_control_point.x(), next_control_point.y()+zmp_offset));
		else if(next_leg_support == footstep_msgs::LeftLegSup)
			foot_step_list.push_back(Eigen::Vector3d(time, next_control_point.x(), next_control_point.y()-zmp_offset));
	}
	time += step_time;
	support_leg_list.push_back(footstep_msgs::BothLeg);
	foot_step_list.push_back(Eigen::Vector3d(time, next_control_point.x(), next_control_point.y()));
	next_leg_support = footstep_msgs::BothLeg;
	#endif
}
#endif
