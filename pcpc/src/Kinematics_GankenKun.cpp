#include "Kinematics.h"
#include <Eigen/SVD>

using namespace MotionControl;

Matrix<double,3,3> Kinematics::Rodrigues(Matrix<double,3,1> a, double q)
{
	return AngleAxisd(q,a).toRotationMatrix();
}

Matrix<double,3,1> Kinematics::rot2omega(Matrix<double,3,3> R)
{
	double alpha = (R(0,0)+R(1,1)+R(2,2)-1)/2;
	double th;
	Matrix<double,3,1> vector_R(Matrix<double,3,1>::Zero());

	if(fabs(alpha-1) < eps)
		return Matrix<double,3,1>::Zero();

	th = std::cos(alpha);
	vector_R << R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1);
	return 0.5*th/std::sin(th)*vector_R;
}

Eigen::Matrix<double,3,3> Kinematics::computeMatrixFromAngles(double roll, double pitch, double yaw)
{
	Eigen::Matrix<double,3,3> R;

	R(0,0) = std::cos(pitch) * std::cos(yaw) - std::sin(roll) * std::sin(pitch) * std::sin(yaw);
	R(0,1) = -std::cos(roll) * std::sin(yaw);
	R(0,2) = std::sin(roll) * std::cos(yaw) + std::sin(roll) * std::cos(pitch) * std::sin(yaw);
	R(1,0) = std::cos(pitch) * std::sin(yaw) + std::sin(roll) * std::sin(pitch) * std::cos(yaw);
	R(1,1) = std::cos(roll) * std::cos(yaw);
	R(1,2) = std::sin(pitch) * std::sin(yaw) - std::sin(roll) * std::cos(pitch) * std::cos(yaw);
	R(2,0) = -std::cos(roll) * std::sin(pitch);
	R(2,1) = std::sin(roll);
	R(2,2) = std::cos(roll) * std::cos(pitch);	

	return R;
}

void Kinematics::computeAnglesFromMatrix(Eigen::Matrix<double,3,3> R, double &roll, double &pitch, double &yaw)
{
	float threshold = 0.001;
	if(abs(R(2,1) - 1.0) < threshold){ // R(2,1) = std::sin(x) = 1の時
		roll 	= M_PI / 2;
		pitch = 0;
		yaw 	= std::atan2(R(1,0), R(0,0));
	}else if(abs(R(2,1) + 1.0) < threshold){ // R(2,1) = std::sin(x) = -1の時
		roll	= - M_PI / 2;
		pitch = 0;
		yaw		= std::atan2(R(1,0), R(0,0));
	}else{
		roll	= std::sin(R(2,1));
		pitch	= std::atan2(-R(2,0), R(2,2));
		yaw		= std::atan2(-R(0,1), R(1,1));
	}
}

vector<int> Kinematics::FindRoute(int to)
{
	vector<int> idx;
	int link_num = to;

	while(link_num != CC)
	{
		if((link_num != RP2) && (link_num != LP2))
			idx.push_back(link_num);
		link_num = ulink[link_num].parent;
	}
	reverse(idx.begin(), idx.end());
	return idx;
}

Matrix<double,6,1> Kinematics::calcVWerr(Link Cref, Link Cnow)
{
	Matrix<double,3,1> perr = Cref.p - Cnow.p;
	Matrix<double,3,3> Rerr = Cref.R - Cnow.R;
	Matrix<double,3,1> werr = Cnow.R * rot2omega(Rerr);
	Matrix<double,6,1> err;

	err << perr,werr;
	return err;
}

void Kinematics::calcForwardKinematics(int rootlink)
{
	if(rootlink == NON) return;
	if(rootlink != CC)
	{
		int parent = ulink[rootlink].parent;
		ulink[rootlink].p = ulink[parent].R * ulink[rootlink].b + ulink[parent].p;
		ulink[rootlink].R = ulink[parent].R * Rodrigues(ulink[rootlink].a, ulink[rootlink].q);
	}
	calcForwardKinematics(ulink[rootlink].sister);
	calcForwardKinematics(ulink[rootlink].child);
}

// 片脚のみの逆運動学
bool Kinematics::calcInverseKinematics(int to, Link target)
{
	Matrix<double,6,6> J;
	Matrix<double,6,1> dq, err;

	const double lambda = 0.5;
	const int iteration = 100;
	
	vector<int> idx = FindRoute(to);
	
	for(int n=0;n<iteration;n++){
		if(to == RR2) calcForwardKinematics(RY);
		if(to == LR2) calcForwardKinematics(LY);
		err = calcVWerr(target, ulink[to]);
		if(err.norm() < eps) return true;
		J = calcJacobian_GankenKun(ulink, idx);
		dq = lambda * (J.inverse() * err);
		for(size_t nn=0;nn<idx.size();nn++){
			int j = idx[nn];
			ulink[j].q += dq(nn);
		}
		if(to == RR2) ulink[RP2].q = -ulink[RP1].q;
		if(to == LR2) ulink[LP2].q = -ulink[LP1].q;
	}
	return false;
}

//両脚の逆運動学を計算する
bool Kinematics::calcInverseKinematicsAll(Link RFLink, Link LFLink)
{
	if(!calcInverseKinematics(RR2, RFLink) || !calcInverseKinematics(LR2, LFLink))
	{
		std::cerr << "Invalid" << std::endl;
		return false;
	}
	
	angle[FOOT_ROLL_R]	= ulink[RR2].q;
	angle[FOOT_PITCH_R]	= ulink[RP3].q + ulink[RP4].q;
	angle[KNEE_R1]		= ulink[RP3].q;
	angle[KNEE_R2]		= ulink[RP1].q;
	angle[LEG_ROLL_R]	= ulink[RR1].q;
	angle[LEG_YAW_R]	= ulink[RY].q;

	angle[FOOT_ROLL_L]	= ulink[LR2].q;
	angle[FOOT_PITCH_L]	= ulink[LP3].q + ulink[LP4].q;
	angle[KNEE_L1]		= ulink[LP3].q;
	angle[KNEE_L2]		= ulink[LP1].q;
	angle[LEG_ROLL_L]	= ulink[LR1].q;
	angle[LEG_YAW_L]	= ulink[LY].q;

	return true;
}

void Kinematics::setJointAngle()
{
	// ToDo : 回転方向をチェック
	ulink[LR2].q =  angle[FOOT_ROLL_L];							// 左足首ロール軸
	ulink[LP4].q = -angle[KNEE_L1    ] + angle[FOOT_PITCH_L];	// 左足首ピッチ軸
	ulink[LP3].q =  angle[KNEE_L1    ];							// 左膝下ピッチ軸（平行リンク）
	ulink[LP2].q = -angle[KNEE_L2    ];							// 左膝上ピッチ軸（平行リンク）
	ulink[LP1].q =  angle[KNEE_L2    ];							// 左股ピッチ軸（平行リンク）
	ulink[LR1].q =  angle[LEG_ROLL_L ];							// 左股ロール軸
	ulink[LY ].q =  angle[LEG_YAW_L  ];							// 左股ヨー軸

	ulink[RR2].q =  angle[FOOT_ROLL_R];							// 右足首ロール軸
	ulink[RP4].q = -angle[KNEE_R1    ] + angle[FOOT_PITCH_R];	// 右足首ピッチ軸
	ulink[RP3].q =  angle[KNEE_R1    ];							// 左膝下ピッチ軸（平行リンク）
	ulink[RP2].q = -angle[KNEE_R2    ];							// 右足膝上ピッチ軸（平行リンク）
	ulink[RP1].q =  angle[KNEE_R2    ];							// 右股ピッチ軸（平行リンク）
	ulink[RR1].q =  angle[LEG_ROLL_R ];							// 右股ロール軸
	ulink[RY ].q =  angle[LEG_YAW_R  ];							// 右股ヨー軸

	ulink[LSP].q =  angle[ARM_PITCH_L];							// 左肩ピッチ軸
	ulink[LSR].q =  angle[ARM_ROLL_L ];							// 左肩ロール軸
	ulink[LER].q =  angle[ELBOW_PITCH_L ];						// 左肘ピッチ軸

	ulink[RSP].q =  angle[ARM_PITCH_R];							// 右肩ピッチ軸
	ulink[RSR].q =  angle[ARM_ROLL_R ];							// 右肩ロール軸
	ulink[RER].q =  angle[ELBOW_PITCH_R ];						// 右肘ピッチ軸

	ulink[SH ].q =  angle[HEAD_YAW   ];							// 首ヨー軸
	ulink[HP ].q =  angle[HEAD_PITCH ];							// 首ピッチ軸（Acceliteは無い）
}
