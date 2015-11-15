/*
 * Gazebo Server for GreenEye Project
 * 
 * Create, delete and update state (reset) of platform by commands from the    FleetController (@Windows)
 */
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <list>
#include <math.h>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "ros/ros.h"


#include "GazeboServer.h"


using namespace std;

struct termios stdio;
struct termios old_stdio;


int m_FleetSize = 1;

udp_server 		*m_RxChannel;
udp_client		*m_TxChannel;

void SetKeyboardInput()
{
	tcgetattr(STDOUT_FILENO,&old_stdio);
	memset(&stdio,0,sizeof(stdio));

	stdio.c_iflag=0;
	stdio.c_oflag=0;
	stdio.c_cflag=0;
	stdio.c_lflag=ISIG;
	stdio.c_cc[VMIN]=1;
	stdio.c_cc[VTIME]=0;
	stdio.c_cc[VINTR]=1;

	tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
	tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);

	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking
}

int main(int argc, char **argv)
{	
	bool 			StopCmd,
					IsKeyActive;
	int 			FleetSize;

	unsigned char 	key = ' ';

	GazeboServer*	m_GazeboServer;

	IsKeyActive = true;
	StopCmd = false;
	ros::init(argc, argv, "gazebo_server");
	ros::NodeHandle nodeHandler;

	// Check if argument of ID is found
	if (argc > 1)
	{
		FleetSize = atoi( argv[1]);
		printf("Gazebo Server for %d platforms Created\n", m_FleetSize);
	}
	else
	{
		printf("Invalid parameters: gazebo_server_node <GroupSize>\n");
		return -1;
	}



	m_GazeboServer = new GazeboServer(nodeHandler);
	m_GazeboServer->SetCommunication();
	m_GazeboServer->SetParameters (FleetSize);

	printf ("press 'r' for reset, 'q' for quit\n");
	// Set keyboard
	SetKeyboardInput();

	while (!StopCmd)
	{
		if (IsKeyActive)
		{
			read(STDIN_FILENO,&key,1);
			switch (key)
			{
				case 'r':
				{
					printf ("Reset pressed\n");
					m_GazeboServer->ResetSimulator();
					break;
				}
				case 't':
				{
					printf ("test pressed\n");
					m_GazeboServer->TestFunction();
					break;
				}
			case 'q':
				printf ("Quit pressed\n");
				StopCmd = true;
				break;
			}

		}
		sleep(1);
		key = ' ';

	}

	tcsetattr(STDOUT_FILENO,TCSANOW,&old_stdio);
	printf("GazeboServer_node closed\n");

	delete(m_GazeboServer);

	return 0;
}
