/*
 * @file		LinkParameter_GankenKun.h
 * @brief		Constant Value of Link for Humanoid Robot
 * @author		Yasuo Hayashibara
 * @date		2017/12/30
 */

#ifndef _LINK_PARAMETER_GANKENKUN_
#define _LINK_PARAMETER_GANKENKUN_

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <limits>
#include <boost/math/constants/constants.hpp>

using namespace std;
using namespace Eigen;

const int NON = -1;
static const double eps = 1e-06;
static const double pi = boost::math::constants::pi<double>();

enum {								//! サーボの番号
	FOOT_ROLL_L 	= 	0,			//! 左足首のロール軸
	KNEE_L1			= 	1,			//! 左足膝下ピッチ軸（平行リンク）
	KNEE_L2			= 	2,			//! 左足膝上ピッチ軸（平行リンク）
	FOOT_PITCH_L 	= 	3,			//! 左足のピッチ軸（シリアルリンク，股のピッチ軸と合わせて足首の向きを変更）
	LEG_ROLL_L		= 	4,			//! 左股のロール軸
	LEG_YAW_L		= 	5,			//! 左股のヨー軸
	FOOT_ROLL_R 	= 	6,			//! 右足首のロール軸
	KNEE_R1			= 	7,			//! 右足膝下ピッチ軸（平行リンク）
	KNEE_R2			= 	8,			//! 右足膝上ピッチ軸（平行リンク）
	FOOT_PITCH_R 	= 	9,			//! 右足のピッチ軸（シリアルリンク，股のピッチ軸と合わせて足首の向きを変更）
	LEG_ROLL_R		= 	10,			//! 右股のロール軸
	LEG_YAW_R		= 	11,			//! 右股のヨー軸
	ARM_PITCH_L		= 	14,			//! 左腕のピッチ軸
	ARM_ROLL_L		= 	15,			//! 左腕のロール軸
	ELBOW_PITCH_L	= 	16,			//! 左肘のピッチ軸
	ARM_PITCH_R		= 	18,			//! 右腕のピッチ軸
	ARM_ROLL_R		= 	19,			//! 右腕のロール軸
	ELBOW_PITCH_R	= 	20,			//! 右肘のピッチ軸
	HEAD_YAW		= 	22,			//! 首のヨー軸
	HEAD_PITCH		= 	23,			//! 首のピッチ軸（Acceliteは無い）
	JOINT_NUM
};

enum {			// リンクの番号
	CAM	,		//! カメラのリンク（光軸を求めるため）
	HT	,		//! 頭のチルト軸からカメラ (Acceliteは固定)
	HP	,		//! 頭のパン軸からチルト軸
	SH	,		//! 肩中心から頭のパン軸
	CS	,		//! 腰から肩中心
	CC	,		//! 腰（基点）
	RY	,		//! 腰から右股ヨー軸
	RR1	,		//! 右股ヨー軸から右股ロール軸
	RP1	,		//! 右股ロール軸から右股ピッチ軸
	RP2	,		//! 股ピッチ軸から右膝上ピッチ軸
	RP3	,		//! 右膝上ピッチ軸から右膝下ピッチ軸
	RP4	,		//! 右膝下ピッチ軸から右足首ピッチ軸
	RR2	,		//! 右足首ピッチ軸から右足首ロール軸
	RF  ,		//! 右足首ロール軸から右足先
	LY	,		//! 腰から左股ヨー軸
	LR1	,		//! 左股ヨー軸から左股ロール軸
	LP1	,		//! 左股ロール軸から左股ピッチ軸
	LP2	,		//! 左股ピッチ軸から左膝上ピッチ軸
	LP3	,		//! 左膝上ピッチ軸から左膝下ピッチ軸
	LP4	,		//! 左膝下ピッチ軸から左足首ピッチ軸
	LR2	,		//! 左足首ピッチ軸から左足首ロール軸
	LF  ,		//!	左足首ロール軸から左足先
	RSP ,		//! 肩中心から右肩ピッチ軸
	RSR	,		//! 右肩ピッチ軸から右肩ロール軸
	RER	,		//! 右肩ロール軸から右肘ピッチ軸
	RH  ,		//! 右肘ピッチ軸から右腕先端
	LSP ,		//! 肩中心から左肩ピッチ軸
	LSR	,		//! 左肩ピッチ軸から左肩ロール軸
	LER	,		//! 左肩ロール軸から左肘ピッチ軸
	LH  ,		//! 左肘ピッチ軸から左腕先端
	LINK_NUM	//! リンクの数	
};

struct link_connect
{
	int sister;
	int child;
	int parent;
};

static const struct link_connect LINK_CONNECT[LINK_NUM] = 
{
	{ NON, NON, HT  },	// CAM : カメラのリンク（光軸を求めるため）
	{ NON, CAM, HP  },	// HT  : 頭のチルト軸からカメラ (Acceliteは固定)
	{ NON, HT , SH  },	// HP  : 頭のパン軸からチルト軸
	{ NON, HP , CS  },	// SH  : 肩中心から頭のパン軸
	{ NON, SH , CC  },	// CS  : 腰から肩中心
	{ RY , CS , NON },	// CC  : 腰（基点）
//	腰 -> 右足先
	{ LY , RR1, CC  },	// RY  : 腰から右股ヨー軸
	{ NON, RP1, RY  },	// RR1 : 右股ヨー軸から右股ロール軸
	{ NON, RP2, RR1 },	// RP1 : 右股ロール軸から右股ピッチ軸
	{ NON, RP3, RP1 },	// RP2 : 右股ピッチ軸から右膝上ピッチ軸
	{ NON, RP4, RP2 },	// RP3 : 右膝上ピッチ軸から右膝下ピッチ軸
	{ NON, RR2, RP3 },	// RP4 : 右膝下ピッチ軸から右足首ピッチ軸
	{ NON, RF , RP4 },	// RR2 : 右足首ピッチ軸から右足首ロール軸
	{ NON, NON, RR2 },	// RF  : 右足首ロール軸から右足先
//	腰 -> 左足先
	{ RSP, LR1, CC  },	// LY  : 腰から左股ヨー軸
	{ NON, LP1, LY  },	// LR1 : 左股ヨー軸から左股ロール軸
	{ NON, LP2, LR1 },	// LP1 : 左股ロール軸から左股ピッチ軸
	{ NON, LP3, LP1 },	// LP2 : 左股ピッチ軸から左膝上ピッチ軸
	{ NON, LP4, LP2 },	// LP3 : 左膝上ピッチ軸から左膝下ピッチ軸
	{ NON, LR2, LP3 },	// LP4 : 左膝下ピッチ軸から左足首ピッチ軸
	{ NON, LF , LP4 },	// LR2 : 左足首ピッチ軸から左足首ロール軸
	{ NON, NON, LR2 },	// LF  : 左足首ロール軸から左足先
//	肩中心 -> 右手先
	{ LSP, RSR, CS  },	// RSP : 肩中心から右肩ピッチ軸
	{ NON, RER, RSP },	// RSR : 右肩ピッチ軸から右肩ロール軸
	{ NON, RH , RSR },	// RER : 右肩ロール軸から右肘ピッチ軸
	{ NON, NON, RER },	// RH  : 右肘ピッチ軸から右腕先端
//	肩中心 -> 左手先
	{ NON, LSR, CS  },	// LSP : 肩中心から左肩ピッチ軸
	{ NON, LER, LSP },	// LSR : 左肩ピッチ軸から左肩ロール軸
	{ NON, LH , LSR },	// LER : 左肩ロール軸から左肘ピッチ軸
	{ NON, NON, LER },	// LH  : 左肘ピッチ軸から左腕先端
};

static const double LinkAxis[LINK_NUM][3] = {
//	{{  前後,   左右,   上下},{前後軸,左右軸,上下軸}} (mm)
//	腰 -> 頭
	{  0.0f,  0.0f,  0.0f},	// CAM : カメラのリンク（光軸を求めるため）
	{  0.0f,  0.0f,  0.0f},	// HT  : 頭のチルト軸からカメラ (Acceliteは固定)
	{  0.0f,  1.0f,  0.0f},	// HP  : 頭のパン軸からチルト軸
	{  0.0f,  0.0f,  1.0f},	// SH  : 肩中心から頭のパン軸
	{  0.0f,  0.0f,  0.0f},	// CS  : 腰から肩中心
	{  0.0f,  0.0f,  0.0f},	// CC  : 腰（基点）
//	腰 -> 右足先
	{  0.0f,  0.0f,  1.0f},	// RY  : 腰から右股ヨー軸
	{  1.0f,  0.0f,  0.0f},	// RR1 : 右股ヨー軸から右股ロール軸
	{  0.0f,  1.0f,  0.0f},	// RP1 : 右股ロール軸から右股ピッチ軸
	{  0.0f,  1.0f,  0.0f},	// RP2 : 右股ピッチ軸から右膝上ピッチ軸
	{  0.0f,  1.0f,  0.0f},	// RP3 : 右膝上ピッチ軸から右膝下ピッチ軸
	{  0.0f,  1.0f,  0.0f},	// RP4 : 右膝下ピッチ軸から右足首ピッチ軸
	{  1.0f,  0.0f,  0.0f},	// RR2 : 右足首ピッチ軸から右足首ロール軸
	{  0.0f,  0.0f,  0.0f},	// RF  : 右足首ロール軸から右足先
//	腰 -> 左足先
	{  0.0f,  0.0f,  1.0f},	// LY  : 腰から左股ヨー軸
	{  1.0f,  0.0f,  0.0f},	// LR1 : 左股ヨー軸から左股ロール軸
	{  0.0f,  1.0f,  0.0f},	// LP1 : 左股ロール軸から左股ピッチ軸
	{  0.0f,  1.0f,  0.0f},	// LP2 : 左股ピッチ軸から左膝上ピッチ軸
	{  0.0f,  1.0f,  0.0f},	// LP3 : 左膝上ピッチ軸から左膝下ピッチ軸
	{  0.0f,  1.0f,  0.0f},	// LP4 : 左膝下ピッチ軸から左足首ピッチ軸
	{  1.0f,  0.0f,  0.0f},	// LR2 : 左足首ピッチ軸から左足首ロール軸
	{  0.0f,  0.0f,  0.0f},	// LF  : 左足首ロール軸から左足先
//	肩中心 -> 右手先
	{  0.0f,  1.0f,  0.0f},	// RSP : 肩中心から右肩ピッチ軸
	{  1.0f,  0.0f,  0.0f},	// RSR : 右肩ピッチ軸から右肩ロール軸
	{  0.0f,  1.0f,  0.0f},	// REP : 右肩ロール軸から右肘ピッチ軸
	{  0.0f,  0.0f,  0.0f},	// RH  : 右肘ピッチ軸から右腕先端
//	肩中心 -> 左手先
	{  0.0f,  1.0f,  0.0f},	// LSP : 肩中心から左肩ピッチ軸
	{  1.0f,  0.0f,  0.0f},	// LSR : 左肩ピッチ軸から左肩ロール軸
	{  0.0f,  1.0f,  0.0f},	// LEP : 左肩ロール軸から左肩ピッチ軸
	{  0.0f,  0.0f,  0.0f},	// LH  : 左肩ピッチ軸から左腕先端
};

static const double LinkPos[LINK_NUM][3] = {
//	{{  前後,   左右,   上下},{前後軸,左右軸,上下軸}} (mm)
//	腰 -> 頭
	{300.0f,   0.0f,   0.0f},	// CAM : カメラのリンク（光軸を求めるため）
	{ 31.4f,   0.0f,   0.0f},	// HT  : 頭のチルト軸からカメラ (Acceliteは固定)
	{  0.0f,   0.0f,  31.3f},	// HP  : 頭のパン軸からチルト軸
	{  0.0f,   0.0f,  21.8f},	// SH  : 肩中心から頭のパン軸
	{ -5.0f,   0.0f, 144.5f},	// CS  : 腰から肩中心
	{  0.0f,   0.0f,   0.0f},	// CC  : 腰（基点）
//	腰 -> 右足先
	{  0.0f, -44.0f,   0.0f},	// RY  : 腰から右股ヨー軸
	{  0.0f,   0.0f, -51.0f},	// RR1 : 右股ヨー軸から右股ロール軸
	{  8.4f,   0.0f,   0.0f},	// RP1 : 右股ロール軸から右股ピッチ軸
	{  0.0f,   0.0f,-100.0f},	// RP2 : 右股ピッチ軸から右膝上ピッチ軸
	{  0.0f,   0.0f, -57.0f},	// RP3 : 右膝上ピッチ軸から右膝下ピッチ軸
	{  0.0f,   0.0f,-100.0f},	// RP4 : 右膝下ピッチ軸から右足首ピッチ軸
	{  0.0f,   0.0f,   0.0f},	// RR2 : 右足首ピッチ軸から右足首ロール軸
	{  0.0f,   0.0f, -43.0f},	// RF  : 右足首ロール軸から右足先
//	腰 -> 左足先
	{  0.0f,  44.0f,   0.0f},	// LY  : 腰から左股ヨー軸
	{  0.0f,   0.0f, -51.0f},	// LR1 : 左股ヨー軸から左股ロール軸
	{  8.4f,   0.0f,   0.0f},	// LP1 : 左股ロール軸から左股ピッチ軸
	{  0.0f,   0.0f,-100.0f},	// LP2 : 左股ピッチ軸から左膝上ピッチ軸
	{  0.0f,   0.0f, -57.0f},	// LP3 : 左膝上ピッチ軸から左膝下ピッチ軸
	{  0.0f,   0.0f,-100.0f},	// LP4 : 左膝下ピッチ軸から左足首ピッチ軸
	{  0.0f,   0.0f,   0.0f},	// LR2 : 左足首ピッチ軸から左足首ロール軸
	{  0.0f,   0.0f, -43.0f},	// LF  : 左足首ロール軸から左足先
//	肩中心 -> 右手先
	{  0.0f, -91.5f,   0.0f},	// RSP : 中心から右肩ピッチ軸
	{  0.0f,   0.0f,   0.0f},	// RSR : 右肩ピッチ軸から右肩ロール軸
	{  0.0f,   0.0f, -84.0f},	// REP : 右肩ロール軸から右肘ピッチ軸
	{  0.0f,   0.0f,-162.5f},	// RH  : 右肘ピッチ軸から右腕先端
//	肩中心 -> 左手先
	{  0.0f,  91.5f,   0.0f},	// LSP : 肩中心から左肩ピッチ軸
	{  0.0f,   0.0f,   0.0f},	// LSR : 左肩ピッチ軸から左肩ロール軸
	{  0.0f,   0.0f, -84.0f},	// LEP : 左肩ロール軸から左肩ピッチ軸
	{  0.0f,   0.0f,-162.5f},	// LH  : 左肩ピッチ軸から左腕先端
};

#endif
