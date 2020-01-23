//#include "MainControl.h"

#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
//	MainControl main_control(argc, argv, 0.01);
//	main_control.updateWalkingMotion();

	ros::init(argc, argv, "pcpc");
	ros::NodeHandle nh;

	ros::Publisher left_ankle_pitch_pub = nh.advertise<std_msgs::Float64>("/left_ankle_pitch_controller/command", 1);
	ros::Publisher left_ankle_roll_pub = nh.advertise<std_msgs::Float64>("/left_ankle_roll_controller/command", 1);
	ros::Publisher left_elbow_pitch_pub = nh.advertise<std_msgs::Float64>("/left_elbow_pitch_controller/command", 1);
	ros::Publisher left_knee_pitch_pub = nh.advertise<std_msgs::Float64>("/left_knee_pitch_controller/command", 1);
	ros::Publisher left_shoulder_pitch_pub = nh.advertise<std_msgs::Float64>("/left_shoulder_pitch_controller/command", 1);
	ros::Publisher left_shoulder_roll_pub = nh.advertise<std_msgs::Float64>("/left_shoulder_roll_controller/command", 1);
	ros::Publisher left_waist_pitch_pub = nh.advertise<std_msgs::Float64>("/left_waist_pitch_controller/command", 1);
	ros::Publisher left_waist_roll_pub = nh.advertise<std_msgs::Float64>("/left_waist_roll_controller/command", 1);
	ros::Publisher left_waist_yaw_pub = nh.advertise<std_msgs::Float64>("/left_waist_yaw_controller/command", 1);

	ros::Publisher right_ankle_pitch_pub = nh.advertise<std_msgs::Float64>("/right_ankle_pitch_controller/command", 1);
	ros::Publisher right_ankle_roll_pub = nh.advertise<std_msgs::Float64>("/right_ankle_roll_controller/command", 1);
	ros::Publisher right_elbow_pitch_pub = nh.advertise<std_msgs::Float64>("/right_elbow_pitch_controller/command", 1);
	ros::Publisher right_knee_pitch_pub = nh.advertise<std_msgs::Float64>("/right_knee_pitch_controller/command", 1);
	ros::Publisher right_shoulder_pitch_pub = nh.advertise<std_msgs::Float64>("/right_shoulder_pitch_controller/command", 1);
	ros::Publisher right_shoulder_roll_pub = nh.advertise<std_msgs::Float64>("/right_shoulder_roll_controller/command", 1);
	ros::Publisher right_waist_pitch_pub = nh.advertise<std_msgs::Float64>("/right_waist_pitch_controller/command", 1);
	ros::Publisher right_waist_roll_pub = nh.advertise<std_msgs::Float64>("/right_waist_roll_controller/command", 1);
	ros::Publisher right_waist_yaw_pub = nh.advertise<std_msgs::Float64>("/right_waist_yaw_controller/command", 1);


	ros::Rate rate(100); //10 ms
	//ros::AsyncSpinner spinner(1);
	//spinner.start();

	std_msgs::Float64 rad;
	while(ros::ok()){

		rad.data; //= -xv_ref.d[LEG_PITCH_L] * (M_PI/180);
		left_ankle_pitch_pub.publish(rad); 
		left_ankle_roll_pub.publish(rad); 
		left_elbow_pitch_pub.publish(rad); 
		left_knee_pitch_pub.publish(rad); 
		left_shoulder_pitch_pub.publish(rad); 
		left_shoulder_roll_pub.publish(rad); 
		left_waist_pitch_pub.publish(rad); 
		left_waist_roll_pub.publish(rad); 
		left_waist_yaw_pub.publish(rad); 

		right_ankle_pitch_pub.publish(rad); 
		right_ankle_roll_pub.publish(rad); 
		right_elbow_pitch_pub.publish(rad); 
		right_knee_pitch_pub.publish(rad); 
		right_shoulder_pitch_pub.publish(rad); 
		right_shoulder_roll_pub.publish(rad); 
		right_waist_pitch_pub.publish(rad); 
		right_waist_roll_pub.publish(rad); 
		right_waist_yaw_pub.publish(rad); 

		ros::spinOnce();
		rate.sleep();
	}

	//spinner.stop();

	//main_control.thread_run();
	return 0;
}
