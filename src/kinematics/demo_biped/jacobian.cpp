#include "jacobian.h"

MatrixXd calcJacobian( Link* ulink, VectorXi idx)
{
	int jsize = idx.size();
	int idxelem = idx(jsize-1);
	Vector3d target = ulink[	idxelem].p;

	MatrixXd J;
	
	J.resize(6,jsize);

	for(int i = 0; i < jsize; i++)
	{
		int j = idx(i);
		Vector3d dw = ulink[j].R * ulink[j].a;	
		Vector3d dp = dw.cross( target - ulink[j].p );

		J(0,i) = dp(0);	
		J(1,i) = dp(1);
		J(2,i) = dp(2);
		J(3,i) = dw(0);
		J(4,i) = dw(1);
		J(5,i) = dw(2);
	}

	return J;
}

