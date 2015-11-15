/*
 * Green Eye Communication Node
 * 1. Receive platformId from command line
 * 2. Create UDP Channel
 * 3. Control velocity
 * 4. Register to position topic
 *
 */
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
#include "GreenEye_Platform.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

using namespace std;



ros::Rate * rate;

GreenEye_Platform* pPlatform;

struct termios stdio;
struct termios old_stdio;

void SetKeyboardInput()
{

	tcgetattr(STDOUT_FILENO,&old_stdio);
	memset(&stdio,0,sizeof(stdio));
/*
	stdio.c_iflag=0;
	stdio.c_oflag=0;
	stdio.c_cflag=0;
	stdio.c_lflag=ISIG;
	stdio.c_cc[VMIN]=1;
	stdio.c_cc[VTIME]=0;
	stdio.c_cc[VINTR]=1;

	tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
	tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
*/
	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking

}
/*
void PositionCallback(const geometry_msgs::PoseStamped& msg)
{
	pPlatform->SetState(msg);
}
*/

void StateCallback (const nav_msgs::Odometry& msg)
{
	pPlatform->SetState(msg);
}

int main(int argc, char **argv)
{
	int				PlatformId;
	string			PlatformIdStr;
	unsigned char key = ' ';

	PlatformId = -1;

	// Check if argument of ID is found
	if (argc > 1)
	{
		PlatformIdStr = argv[1];
		PlatformId = atoi( argv[1]);
		if (PlatformId > 0)
			printf("ge_platform_node%d Created\n", PlatformId);
		else
		{
			printf("ID must be 1 or greater\n");
			return -1;
		}
	}
	else
	{
		printf("Invalid parameters: ge_comm_node <Id>\n");
		return -1;
	}

// Set keyboard
	SetKeyboardInput();

	//Initialize the node and connect to master
	ros::init(argc, argv, "ge_com" + PlatformIdStr);
	geometry_msgs::PoseStamped msg;
	ros::NodeHandle nodeHandler;

	pPlatform = new GreenEye_Platform(PlatformId);

	//generate a node handler to handle all messages
	pPlatform->m_VelPublish = nodeHandler.advertise<geometry_msgs::Twist>("/uav" + PlatformIdStr +"/cmd_vel", 1024);

	//ros::Subscriber uav_sub = nodeHandler.subscribe("/uav" + PlatformIdStr + "/ground_truth_to_tf/pose", 1024, &PositionCallback);
	ros::Subscriber uav_sub = nodeHandler.subscribe("/uav" + PlatformIdStr + "/ground_truth/state", 1024, &StateCallback);

	ros::Rate * rate=new ros::Rate(5);
	ros::spinOnce();
	(*rate).sleep();

	// m_Client->send(TxBuffer, BUFFER_SIZE);

	bool IsExit = false;
	bool IsPause = false;

	while (!IsExit && ros::ok())
	{
		pPlatform->Run();

		read(STDIN_FILENO,&key,1);
		switch (key)
		{
		case 'q':
			printf ("Stop pressed\n");
			pPlatform->StopCommand(true);
			IsExit = true;
			break;
		case 's':
		//case 'S':
			IsPause = !IsPause;
			if (IsPause)
			{
				printf ("Stop pressed\n");
				pPlatform->StopCommand(false);
			}
			else
			{
				printf ("Play pressed\n");
				pPlatform->ResumeCommand();
			}
			break;
		case 'p':
		{
			IPlatform_State state = pPlatform->GetState();
			//geometry_msgs::PoseStamped state = pPlatform->GetState();
			printf ("Pos: (%f,%f,%f) \n",state.Position.North, state.Position.East, state.ASL );
			break;
		}
		case 't':	// move object to another position
			{
				printf ("Test reset movement\n");

				//ros::Publisher reset_pos = reset_pos = nodeHandler.advertise<gazebo_msgs::SetModelStateRequest_>  ("gazebo/set_model_state", 1024);
				//gazebo_msgs::SetModelState msg;
				ros::ServiceClient client = nodeHandler.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
				  gazebo_msgs::SetModelState setmodelstate;
				  gazebo_msgs::ModelState modelstate;
				  modelstate.model_name = "uav" + PlatformIdStr;
				  modelstate.reference_frame = "world";

				  geometry_msgs::Twist model_twist;
				  model_twist.linear.x = -1.0;
				  model_twist.linear.y = 1.0;
				  model_twist.linear.z = 0.0;
				  model_twist.angular.x = 0.25;
				  model_twist.angular.y = 0.0;
				  model_twist.angular.z = 0.0;
				  modelstate.pose.position.x = 0;
				  modelstate.pose.position.y = modelstate.pose.position.y + 2;
				  modelstate.pose.position.z = 0;
				  modelstate.twist = model_twist;

				  setmodelstate.request.model_state = modelstate;
				  client.call(setmodelstate);




				//reset_pos.publish(msg);
				break;
			}
		}
		key = ' ';

		ros::spinOnce();
		(*rate).sleep();
	}

	pPlatform->StopCommand(true);
	delete (pPlatform);

	tcsetattr(STDOUT_FILENO,TCSANOW,&old_stdio);

	return 0;
}
