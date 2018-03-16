#include "jacobian.h"

MatrixXd calcJacobian( Link *ulink, vector<int> idx )
{
	size_t jsize = idx.size();
	Matrix<double,3,1> target = ulink[idx.back()].p;
	MatrixXd J;
	
	J.resize(6,jsize);

	for(size_t i=0; i<jsize; i++)
	{
		int j = idx[i];
		Matrix<double,3,1>a = ulink[j].R * ulink[j].a;
		Matrix<double,3,1>b = a.cross(target - ulink[j].p);
		J(0,i) = b(0);
		J(1,i) = b(1);
		J(2,i) = b(2);
		J(3,i) = a(0);
		J(4,i) = a(1);
		J(5,i) = a(2);
	}
	return J;
}
