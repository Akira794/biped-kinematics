#ifndef __LINK_PARAMETER_H__
#define __LINK_PARAMETER_H__

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

//Biped
enum {//! servo number
    
	FOOT_ROLL_L  = 0,
	KNEE_L1      = 1,
	KNEE_L2      = 2,
	FOOT_PITCH_L = 3,
	LEG_ROLL_L   = 4,
	LEG_YAW_L    = 5,
	FOOT_ROLL_R  = 6,
	KNEE_R1      = 7,
	KNEE_R2      = 8,
	FOOT_PITCH_R = 9,
	LEG_ROLL_R   = 10,
	LEG_YAW_R    = 11,
	JOINT_NUM
};

enum { //! link number

	BASE =0,
	RY   =1,
	RR1  =2,
	RP1  =3,
	RP2  =4,
	RP3  =5,
	RP4  =6,
	RR2  =7,
	RF   =8,
	LY   =9,
	LR1  =10,
	LP1  =11,
	LP2  =12,
	LP3  =13,
	LP4  =14,
	LR2  =15,
	LF   =16,
	LINK_NUM
};

struct link_connect
{
	int sister;
	int child;
	int parent;
};

/*
static const int parent[LINK_NUM] = { -1, 0,  1,  2,  3,  4,  5,  6,  7,  0,  9, 10, 11, 12, 13, 14, 15 };
static const int child[LINK_NUM]  = {  1, 2,  3,  4,  5,  6,  7,  8, -1, 10, 11, 12, 13, 14, 15, 16, -1 };
static const int sister[LINK_NUM] = { -1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }; 
*/
static const struct link_connect LINK_CONNECT[LINK_NUM] = 
{
	{ -1, 1,  -1},

	{  9, 2, 0 },
	{ -1, 3, 1 },
	{ -1, 4, 2 },
	{ -1, 5, 3 },
	{ -1, 6, 4 },
	{ -1, 7, 5 },
	{ -1, 8, 6 },
	{ -1, -1,7 },

	{ -1, 10, 0 },
	{ -1, 11, 9 },
	{ -1, 12, 10},
	{ -1, 13, 11},
	{ -1, 14, 12},
	{ -1, 15, 13},
	{ -1, 16, 14},
	{ -1, -1, 15},

};

static const int LinkAxis[LINK_NUM][3] = {

	/* WAIST */
	{0,0,0},

	/* RLEG */	
	{0,0,1},
	{1,0,0},
	{0,1,0},
	{0,1,0},
	{0,1,0},
	{0,1,0},
	{1,0,0},
	{0,0,0},

	/* LLEG */      
	{0,0,1},
	{1,0,0},
	{0,1,0},
	{0,1,0},
	{0,1,0},
	{0,1,0},
	{1,0,0},
	{0,0,0},

};

static const double LinkPos[LINK_NUM][3] = {

		/* BASE */
	{0.0f, 0.0f, 0.00f},
		/* RLEG */            
	{0.0f, -44.0f,  0.00f},
	{0.0f,  0.0f,  -51.0f},
	{0.0f,  0.0f,    0.0f},
	{0.0f,  0.0f, -100.0f},
	{0.0f,  0.0f,  -57.0f},
	{0.0f,  0.0f, -100.0f},
	{0.0f,  0.0f,  -43.0f},
	{43.0f, 0.0f,     0.0f},
		/* LLEG */
	{0.0f,  44.0f,  0.00f},
	{0.0f,  0.0f,  -51.0f},
	{0.0f,  0.0f,    0.0f},
	{0.0f,  0.0f, -100.0f},
	{0.0f,  0.0f,  -57.0f},
	{0.0f,  0.0f, -100.0f},
	{0.0f,  0.0f,  -43.0f},
	{43.0f,  0.0f,   0.0f},
};

#endif
