#ifndef __DEMO_BIPED_LINK_H__
#define __DEMO_BIPED_LINK_H__

#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/math/constants/constants.hpp>
using namespace std;
using namespace Eigen;

static const double eps = 1e-9;
static const double pi = boost::math::constants::pi<double>();

/* Demo biped */
static const string joint_name[] = {
	"CC",
	"RLEG_J0",
	"RLEG_J1",
	"RLEG_J2",
	"RLEG_J3",
	"RLEG_J4",
	"RLEG_J5",
	"LLEG_J0",
	"LLEG_J1",
	"LLEG_J2",
	"LLEG_J3",
	"LLEG_J4",
	"LLEG_J5"
};

enum {
	CC = 0,
	RLEG_J0,
	RLEG_J1,
	RLEG_J2,
	RLEG_J3,
	RLEG_J4,
	RLEG_J5,

	LLEG_J0,
	LLEG_J1,
	LLEG_J2,
	LLEG_J3,
	LLEG_J4,
	LLEG_J5,
	JOINT_NUM
};

static const int parent[JOINT_NUM] = { -1, 0,  1,  2,  3,  4,  5,  0,  7,  8,  9, 10, 11 };
static const int child[JOINT_NUM]  = {  1, 2,  3,  4,  5,  6, -1,  8,  9, 10, 11, 12, -1 };
static const int sister[JOINT_NUM] = { -1, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }; 
static const double joint_mass[JOINT_NUM] = { 10.0, 5.0, 1.0, 5.0, 1.0, 6.0, 2.0, 5.0, 1.0, 5.0, 1.0, 6.0, 2.0};
static const int Axis[JOINT_NUM][3] = {

	/* WAIST */
				{0,0,0},

	/* RLEG */	
				{0,0,1},
				{1,0,0},
				{0,1,0},
				{0,1,0},
				{0,1,0},
				{1,0,0},

        /* LLEG */      
        {0,0,1},
        {1,0,0},
        {0,1,0},
        {0,1,0},
        {0,1,0},
        {1,0,0},

};

static const double Pos[JOINT_NUM][3] = {

        /* WAIST */
        {0.0f, 0.0f, 0.00f},

        /* RLEG */            
        {0.0f, -44.0f,  0.00f},
        {0.0f,  0.0f,  -42.8f},
        {15.0f,  0.0f,    0.0f},
        {0.0f,  0.0f, -108.0f},
        {0.0f,  0.0f, -108.0f},
        {0.0f,  0.0f, -10.00f},

        /* LLEG */
        {0.0f,  44.0f,  0.00f},
        {0.0f,  0.0f,  -42.0f},
        {15.0f,  0.0f,  0.0f},
        {0.0f,  0.0f, -108.0f},
        {0.0f,  0.0f, -108.0f},
        {0.0f,  0.0f, -10.0f},

};

#endif
