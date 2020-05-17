//Linkの確認

#include "Link.h"

MotionControl::Link ulink[LINK_NUM];

int main(int argc, char *argv[])
{

	SetJointInfo(ulink);

	for(int i=0;i<6;i++){
		std::cout << ulink[i].child << std::endl;
	}

	return 0;
}
