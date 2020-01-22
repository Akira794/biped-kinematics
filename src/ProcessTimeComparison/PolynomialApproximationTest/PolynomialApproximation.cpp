#include <iostream>
#include <stdio.h>
#include <cmath>
#include <string>
#include <boost/timer/timer.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

const float start_walk_x[] = {0.0094961, 0.129835, -0.0556214, 0.0352379, 0.00031604, 9.61663e-07};
const float start_walk_y[] = {0.0173041, -0.651063, 0.274715, -0.166368, -0.00143615, -4.8335e-06};

const float normal_walk_left_x[] = {0.728176, 0.0263143, 0.200574, 0.0639403, 0.0274002, 0.00378723};
const float normal_walk_left_y[] = {0.56449, 1.04444, -0.498798, 0.544836, -0.108628, -0.0175238};

const float normal_walk_right_x[] = {1.67415, -1.48821, 1.41732, -0.546305, 0.225423, 0.0317736};
const float normal_walk_right_y[] = {0.311494, -1.92335, 1.30056, -0.877178, 0.238402, 0.00974532};

const float stop_walk_x[] = {1.23413, -1.51694, 1.07807, -0.54138, 0.179911, 0.0864443};
const float stop_walk_y[] = {-3.16321, 3.42743, -1.26965, 0.946102, -0.202011, 0.0300985};

Vector2f start_walk_pattern(int time)
{
	Vector2f com_pos;

	com_pos(0) = start_walk_x[0]*pow(time,5) + start_walk_x[1]*pow(time,4) + start_walk_x[2]*pow(time,3) + start_walk_x[3]*pow(time,2) + start_walk_x[4]*pow(time,1) + start_walk_x[5];
	com_pos(1) = start_walk_y[0]*pow(time,5) + start_walk_y[1]*pow(time,4) + start_walk_y[2]*pow(time,3) + start_walk_y[3]*pow(time,2) + start_walk_y[4]*pow(time,1) + start_walk_y[5];

	return com_pos;
}

Vector2f normal_walk_left_pattern(int time)
{
	Vector2f com_pos;

	com_pos(0) = normal_walk_left_x[0]*pow(time,5) + normal_walk_left_x[1]*pow(time,4) + normal_walk_left_x[2]*pow(time,3) + normal_walk_left_x[3]*pow(time,2) + normal_walk_left_x[4]*pow(time,1) + normal_walk_left_x[5];
	com_pos(1) = normal_walk_left_y[0]*pow(time,5) + normal_walk_left_y[1]*pow(time,4) + normal_walk_left_y[2]*pow(time,3) + normal_walk_left_y[3]*pow(time,2) + normal_walk_left_y[4]*pow(time,1) + normal_walk_left_y[5];

	return com_pos;
}

Vector2f normal_walk_right_pattern(int time)
{
	Vector2f com_pos;

	com_pos(0) = normal_walk_right_x[0]*pow(time,5) + normal_walk_right_x[1]*pow(time,4) + normal_walk_right_x[2]*pow(time,3) + normal_walk_right_x[3]*pow(time,2) + normal_walk_right_x[4]*pow(time,1) + normal_walk_right_x[5];
	com_pos(1) = normal_walk_right_y[0]*pow(time,5) + normal_walk_right_y[1]*pow(time,4) + normal_walk_right_y[2]*pow(time,3) + normal_walk_right_y[3]*pow(time,2) + normal_walk_right_y[4]*pow(time,1) + normal_walk_right_y[5];

	return com_pos;
}

Vector2f stop_walk_pattern(int time)
{
	Vector2f com_pos;

	com_pos(0) = stop_walk_x[0]*pow(time,5) + stop_walk_x[1]*pow(time,4) + stop_walk_x[2]*pow(time,3) + stop_walk_x[3]*pow(time,2) + stop_walk_x[4]*pow(time,1) + stop_walk_x[5];
	com_pos(1) = stop_walk_y[0]*pow(time,5) + stop_walk_y[1]*pow(time,4) + stop_walk_y[2]*pow(time,3) + stop_walk_y[3]*pow(time,2) + stop_walk_y[4]*pow(time,1) + stop_walk_y[5];


	return com_pos;
}

enum WalkSequence{
    start_walk=0,
    normal_walk=1,
    stop_walk,
};

int main(int argc, char *argv[])
{
	FILE *footpos_fp;
	Vector2f modi_com_pos;
	WalkSequence walk_sequence;

	int step_count=1;
	int max_step=8;
	static int supleg=1;
	int timer=1;

	bool result=true;

	Vector2f refzmp, com_pos;
	Matrix2f rot;
  rot=Rotation2Df(0.15708);

	footpos_fp = fopen("footpos_pattern.csv", "w");

	walk_sequence= start_walk;
	boost::timer::cpu_timer prosess_timer;
	while(result){
		switch(walk_sequence){
		    case start_walk:
			    com_pos = start_walk_pattern(timer);
			    fprintf(footpos_fp, "%f %f\n", com_pos(0), com_pos(1));
			    timer++;
			        if(52 < timer){
				        walk_sequence = normal_walk;
				        timer = 1;
			        }
			    break;
			case normal_walk:
			    if(supleg == 0){
				    cout<<supleg<<endl;
				    modi_com_pos = normal_walk_left_pattern(timer);


			    }else if(supleg == 1){
				    cout<<supleg<<endl;
				    modi_com_pos = normal_walk_right_pattern(timer);


			    }
					com_pos = rot*modi_com_pos;

			    fprintf(footpos_fp, "%f %f\n", com_pos(0), com_pos(1));
			    timer++;
			    if(34 < timer){
				    timer = 1;
				    step_count++;
				    if(max_step < step_count){
					    walk_sequence = stop_walk;
					    timer = 1;
				    }
				    if(supleg == 0)
					    supleg = 1;
				    else if(supleg == 1)
					    supleg = 0;
			    }
			    break;
			case stop_walk:
			    com_pos = stop_walk_pattern(timer);
			    fprintf(footpos_fp, "%f %f\n", com_pos(0), com_pos(1));
			    timer++;
			    if(123 < timer)
				    result = false;
				break;
		}
	}
	std::string prosess_time = prosess_timer.format(9, "経過時間：%w秒\n"
	                                "ユーザーCPU処理時間：%u秒\n"
									 "プロセスCPU処理時間：%s秒");
	std::cout << prosess_time << std::endl;
	fclose(footpos_fp);
	return 0;
}
