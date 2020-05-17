#include "MainControl.h"

int main(int argc, char *argv[])
{
	MainControl main_control(argc, argv, 0.01);
	main_control.thread_run();
	return 0;
}
