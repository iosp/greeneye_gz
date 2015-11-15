/*
Create number of platforms and create topic for each of them using sprintf function
To move platform, the user should select number of platform and then direction to move
*/


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
//#include "std_msgs/String.h"

#include <stdio.h>
//#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
//#include <stdlib.h>


using namespace std;

ros::Rate * rate;

#define 	MAX_UAVS	10


int main(int argc, char **argv)
{

  //Initialize the node and connect to master
  ros::init(argc, argv, "fleet_udp_node");

  //generate a node handler to handle all messages
  ros::NodeHandle n;

//  descisionMaking_pub = new ros::Publisher(n.advertise<std_msgs::String>("/decision_making/events", 1000));
  ros::Publisher Selected_Pub;



  //string TopicName;
  char TopicName[100];
  ros::Publisher Pub_uavs[MAX_UAVS];
  for (int i = 0; i < MAX_UAVS; i++)
  {
	  sprintf(TopicName,"/uav%d/cmd_vel", i);

	  Pub_uavs[i] = n.advertise<geometry_msgs::Twist>(TopicName, 1000);
  }
  printf("Create keyboard control for %d uavs \n", MAX_UAVS);
  printf ("To Control uav, first select its id and then the direction command \n");

  Selected_Pub = Pub_uavs[1];
  ros::Rate * rate=new ros::Rate(5);
  ros::spinOnce();
  (*rate).sleep();
  double supporter_max=1.6;
  double supporter_min=0;

  double loader_max=1.0;
  double loader_min=-1.0;

  double bracket_max=1.0;
  double bracket_min=-1.0;

  double supporter_val=0;
  double loader_val=0;
  double bracket_val=0;

  bool IsStop = false;


  //structs to hold the shell buffer
  struct termios stdio;
  struct termios old_stdio;

  unsigned char c='D';
  char key;
  tcgetattr(STDOUT_FILENO,&old_stdio);

  memset(&stdio,0,sizeof(stdio));
  stdio.c_iflag=0;
  stdio.c_oflag=0;
  stdio.c_cflag=0;
  stdio.c_lflag=ISIG;
  stdio.c_cc[VMIN]=1;
  stdio.c_cc[VTIME]=0;
  //stdio.c_cc[VINTR]=1;
  tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
  tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking


  geometry_msgs::Twist twistMsg;

//  std_msgs::String dm_msg;
//  dm_msg.data="/Teleoperation";
//  descisionMaking_pub->publish(dm_msg);
  ros::spinOnce();
  (*rate).sleep();

  while (ros::ok() && !IsStop)
  {

	  //fgets (&key, 1, stdin);
	  //printf("Key");
	//while (!IsKeyPressed)
	//{
		// Select the uav to move
	while(read(STDIN_FILENO,&c,1)>0)
	{
	}


	if (c >= '0' && c <= '9')
	{
		int j;
		j = c - '0';
		Selected_Pub = Pub_uavs[j];
		printf ("uav%d selected\n",j);
	}
	else
	{
		switch (c)
		{
			case 'w':
			    twistMsg.linear.z=0.5;
			    Selected_Pub.publish(twistMsg);
			    break;
			case 'x':
				//twistMsg.angular.z=-0.5;
				twistMsg.linear.z=-0.5;
			    Selected_Pub.publish(twistMsg);
			    break;
			case 's':
				twistMsg.linear.x = twistMsg.linear.y = twistMsg.linear.z=0;
				Selected_Pub.publish(twistMsg);
				break;
			case 'a':
				twistMsg.linear.x = -0.5;
				Selected_Pub.publish(twistMsg);
				break;
			case 'd':
				twistMsg.linear.x = 0.5;
				Selected_Pub.publish(twistMsg);
				break;
			case 'z':
				twistMsg.linear.y = -0.5;
				Selected_Pub.publish(twistMsg);
				break;
			case 'e':
				twistMsg.linear.y = 0.5;
				Selected_Pub.publish(twistMsg);
				break;
			case 'q':
				IsStop = true;
				break;
		}

	}
	c = ' ';
	ros::spinOnce();
	(*rate).sleep();

}
  // stop all uavs
  twistMsg.linear.x = twistMsg.linear.y = twistMsg.linear.z = 0;
  for (int i = 0; i < MAX_UAVS; i++)
    {
  	  Pub_uavs[i].publish(twistMsg);

    }

  ros::spinOnce();
  (*rate).sleep();
  tcsetattr(STDOUT_FILENO,TCSANOW,&old_stdio);

  return 0;
}
