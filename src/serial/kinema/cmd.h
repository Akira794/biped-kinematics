#ifndef __CMD_H__
#define __CMD_H__

#include <cstdio>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

int kbhit( void );
void InputKey( int* );

#endif
