#ifndef _CONTROLZMP_HRZ_U__H_ 
#define _CONTROLZMP_HRZ_U__H_

#include "misc.h"

using namespace Eigen;

class horizon_u
{
	private:
		double _vd;			/* referential COM velocity */
		double _q1, _q2;	/* system poles */
		double *_kappa;		/* curvature of referential orbit */
		double *_zeta;		/* interference from vertical motion */
	public:
		 horizon_u() : dt(_dt){}
		~horizon_u(){}
		double CalcSimZMP( Matrix<double,2,1> delta,  Matrix<double,2,1> vel );
		double CalcRegZMP( Matrix<double,2,1> delta,  Matrix<double,2,1> vel );
};


