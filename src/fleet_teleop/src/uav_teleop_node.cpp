#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/String.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>

using namespace std;

#define		MAX_VELOCITY		1.0

ros::Rate * rate;

/*
6/7/15 - DF 
Publish uav/cmd_vel using input from the keyboard
i - up
k - down
q - quit
*/

int main(int argc, char **argv)
{
  int platformId = 0;
  string Id;
  if (argc > 1)
	  Id = argv[1];
  else
  {
	  printf("No valid args: [id]\n");
	  return -1;
  }
  
  double 	CurrentVel = 0.2;
	
  //signal(SIGINT, siginthandler);
  //Initialize the node and connect to master
  ros::init(argc, argv, "uav_keyboad_node");
  printf ("Navigate using Keypad and WASD  \n");
  printf ("5 - Stop, 9 - UP, 3 - DOWN \n press q to quit\n");

  //generate a node handler to handle all messages
  ros::NodeHandle n;

  // convert platformId to topic name
  char topicName[10];
  snprintf(topicName, 10, "/uav%d", platformId);
  string pub_str = (string)topicName;

pub_str = "/uav" + Id;

  ros::Publisher wheelsrate_pub = n.advertise<geometry_msgs::Twist>(pub_str + "/cmd_vel", 1000);


  ros::Rate * rate=new ros::Rate(5);
  ros::spinOnce();
  (*rate).sleep();


  //structs to hold the shell buffer
  struct termios stdio;
  struct termios old_stdio;

  unsigned char c='D';
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



  geometry_msgs::Vector3 vectorMsg;
  bool stop=false;

//  std_msgs::String dm_msg;
//  dm_msg.data="/Teleoperation";
//  descisionMaking_pub->publish(dm_msg);
  ros::spinOnce();
  (*rate).sleep();
  while (ros::ok() && c!='q' && !stop)
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

	geometry_msgs::Twist twistMsg;
	while(read(STDIN_FILENO,&c,1)>0){}


    if (c!='q' && c!=' ')
    {
	int i=c;
	if(i==3)
	  stop=true;
	
    	switch(c){
    	case 'w':
    		twistMsg.linear.x = CurrentVel;
    		twistMsg.angular.z = 0;
    		wheelsrate_pub.publish(twistMsg);
    		break;
    	case 'a':
    		twistMsg.linear.x = 0.2;
    		twistMsg.angular.z = 0.3;
    		wheelsrate_pub.publish(twistMsg);
    		break;
    	case 'd':
    		twistMsg.linear.x = 0.2;
    		twistMsg.angular.z = -0.3;
    		wheelsrate_pub.publish(twistMsg);
    		break;
    	case 's':
    		twistMsg.linear.x = 0;
    		twistMsg.angular.z = 0;
    		wheelsrate_pub.publish(twistMsg);
    		break;

    		case '6':
    			twistMsg.linear.x = CurrentVel;
    			wheelsrate_pub.publish(twistMsg);
    			break;
    		case '4':
    			twistMsg.linear.x= -CurrentVel;
    			wheelsrate_pub.publish(twistMsg);
    			break;
    		case '8':
    			twistMsg.linear.y=CurrentVel;
    			wheelsrate_pub.publish(twistMsg);
    			break;
    		case '2':
    			twistMsg.linear.y=-CurrentVel;
    			wheelsrate_pub.publish(twistMsg);
    			break;
    		case '9':
    			twistMsg.linear.z=CurrentVel;
    			wheelsrate_pub.publish(twistMsg);
    			break;
    		case '3':
    			twistMsg.linear.z=-CurrentVel;
    			wheelsrate_pub.publish(twistMsg);
    			break;
    		case '+':
    			CurrentVel = min (MAX_VELOCITY, CurrentVel+0.1);
    			wheelsrate_pub.publish(twistMsg);
    			break;
    		case '-':
    		    CurrentVel = max (0.0, CurrentVel-0.1);
    		    wheelsrate_pub.publish(twistMsg);
    		    break;
    		case '5':
    			twistMsg.linear.x=0;
    			twistMsg.linear.y=0;
    			twistMsg.linear.z=0;
    			wheelsrate_pub.publish(twistMsg);
    			break;

    		
    		default:
    			break;
    	}
        ros::spinOnce();
        (*rate).sleep();
        c=' ';
    }
    else
    {
    	  //stop all activity
    	//twistMsg.linear.x=0;
    	//twistMsg.angular.x=0;
		//wheelsrate_pub.publish(twistMsg);
	    ros::spinOnce();
    	(*rate).sleep();
    }
  }

//  dm_msg.data="/Autonomy";
//  descisionMaking_pub->publish(dm_msg);
  ros::spinOnce();
  (*rate).sleep();
  tcsetattr(STDOUT_FILENO,TCSANOW,&old_stdio);

  return 0;
}
