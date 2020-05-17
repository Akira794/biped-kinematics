#ifndef _LINK_PARAMETER_
#define _LINK_PARAMETER_

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

//Manipulator
static const string joint_name[] = {
	"CC",
	"JY1",
	"JP1",
	"JP2",
	"JY2",
	"JP3",
	"JY3",
	"JH",
};

enum {							//! サーボの番号
	J0=0,
	J1=1,
	J2=2,
	J3=3,
	J4=4,
	J5=5,
	JOINT_NUM
};

enum {			// リンクの番号
	CC,	//! 腰（基点）
	JY1,
	JP1,
	JP2,
	JY2,
	JP3,
	JY3,
	JH,       //! 先端
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
	{ NON, JY1, NON },	// CC  : 腰から肩中心
//	腰 -> 右足先
	{ NON, JP1,  CC },	// JY1
	{ NON, JP2, JY1 },	// JP1
	{ NON, JY2, JP1 },	// JP2
	{ NON, JP3, JP2 },	// JY2
	{ NON, JY3, JY2 },	// JP3
	{ NON,  JH, JP3 },	// JY3
	{ NON, NON, JY3 },	// JH
};

static const double LinkAxis[LINK_NUM][3] = {
//	{{  前後,   左右,   上下},{前後軸,左右軸,上下軸}} (mm)

	{  0.0f,  0.0f,  0.0f},	// CC  : （基点）
	{  0.0f,  0.0f,  1.0f},	// JY1
	{  0.0f,  1.0f,  0.0f},	// JP1
	{  0.0f,  1.0f,  0.0f},	// JP2
	{  0.0f,  0.0f,  1.0f},	// JY2
	{  0.0f,  1.0f,  0.0f},	// JP3
	{  0.0f,  0.0f,  1.0f},	// JY3
	{  1.0f,  0.0f,  0.0f},	// JH
};

static const double LinkPos[LINK_NUM][3] = {
//	{{  前後,   左右,   上下},{前後軸,左右軸,上下軸}} (mm)

	{  0.0f,   0.0f,   0.0f},	// CC  : （基点）
	{  0.0f, 	 0.0f,   70.0f},	// JY1
	{  50.0f, 0.0f, 30.0f},	// JP1
	{  0.0f,   0.0f,   100.0f},	// JP2
	{  0.0f,   0.0f,	0.0f},	// JY2
	{  0.0f,   0.0f, 	100.0f},	// JP3
	{  0.0f,   0.0f,   0.0f},	// JY3
	{  0.0f,   0.0f,   1.0f},	// JH
};

#endif
