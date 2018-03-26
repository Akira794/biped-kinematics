#include "cmd.h"

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

void InputKey( int *cmd )
{
	char key;
	if(kbhit()){
		key = getchar();
		if(key == 'x')
		{
			printf( "\n Reset\n");
			*cmd = -2;
		}else if(key == 'f'){
			printf( "\n Forward");
			*cmd = 1;
		}else if(key == 'j'){
			printf( "\n Back");
			*cmd = 2;
		}else if(key == 'd'){
		  	printf( "\n left");
			*cmd = 3;
		}else if(key == 'k'){
		  	printf( "\n right");
			*cmd = 4;
		}else if(key == 's'){
			printf( "\n Up");
			*cmd = 5;
		}else if(key == 'l'){
			printf( "\n Down");
			*cmd = 6;
		}else if(key == 'r'){
		  	printf( "\n roll+");
			*cmd = 7;
		}else if(key == 'u'){
		  	printf( "\n roll-");
			*cmd = 8;
		}else if(key == 'e'){
			printf( "\n pitch+");
			*cmd = 9;
		}else if(key == 'i'){
			printf( "\n pitch-");
			*cmd = 10;
		}else if(key == 'w'){
		  	printf( "\n yaw+");
			*cmd = 11;
		}else if(key == 'o'){
		  	printf( "\n yaw-");
			*cmd = 12;
		}else if(key == 'c'){
				printf("\n");
			*cmd = 13;
		}else if(key == 'n'){
				printf("\n");
			*cmd =14;
		}else if(key == 'z'){
				printf("\n");
			*cmd =15;
		}else if(key == 'm'){
				printf("\n");
			*cmd =16;
		}else if(key == 'q'){
			printf( "\n Exit \n\n");
			*cmd = -1;
		}else{
			printf( "\nno key\n");
		}
	}
}

