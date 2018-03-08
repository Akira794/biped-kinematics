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

		if(key == 'x'){

			printf( "\nReset");

			*cmd = 10;
		}
		else if(key =='s'){

			printf( "\nLeft");

			*cmd = 1;
		}

		else if(key=='f'){

      printf("\nRight");

			*cmd = 2;
    }

		else if(key=='d'){

      printf("\nBack");

      *cmd = 3;
		}

		else if(key =='e'){

      printf( "\nForward");

			*cmd = 4;
    }

		else if(key=='i'){

      printf("\nUp");

      *cmd = 5;
    }

    else if(key =='k'){

      printf( "\nDown");

      *cmd = 6;
    }


		else if(key =='u'){
			
			*cmd = 7;
		}

		else if(key =='j'){
		
			*cmd = 8;
		}

		else if(key =='q'){
			printf( "\nEXIT\n");
			*cmd = -1;	
		}
		else{
			printf(" ");

		}
	}

}

