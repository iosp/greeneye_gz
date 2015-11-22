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
#include "math.h"

#include "GreenEye_Platform.h"

#include <time.h>


using namespace std;


GreenEye_Platform::GreenEye_Platform(int a_Id)
{
	m_RxChannel = 0;
	m_TxChannel = 0;
	m_Id = a_Id;
	Init();
}

GreenEye_Platform::~GreenEye_Platform()
{
	if (m_RxChannel != 0)
		delete (m_RxChannel);
	if (m_TxChannel != 0)
		delete (m_TxChannel);

	printf("ge_comm_node%d closed\n" , m_Id);
}

bool GreenEye_Platform::Init()
{
	int 	TxPort,
			RxPort;

	// Set State of platform
	m_State.Energy = 100;
	m_State.DeployState = 0;
	m_State.FailureState = 0;
	m_State.Type = 1;
	m_State.m_MissionState = MISSION_STATE_HOME;
	m_SimState = SIMULATOR_STOP;//SIMULATOR_PLAY;

	if (m_RxChannel != 0)
		delete (m_RxChannel);
	if (m_TxChannel != 0)
		delete (m_TxChannel);

	TxPort = BASE_PORT_TX + m_Id;
	RxPort = BASE_PORT_RX + m_Id;
	//m_RxChannel  = new udp_server(IP_ADDR, RxPort);
	//m_TxChannel = new udp_client(IP_ADDR, TxPort);
	m_RxChannel  = new udp_server(PLATFORM_IP, RxPort);
	m_TxChannel = new udp_client(FLEET_CONTROOLER_IP, TxPort);

	m_RxChannel->Connect(false);

	m_MsgTxCounter = 0;

	m_FlightState = FLIGHT_WAIT;
	//m_FlightMode = FLIGHT_MODE_MANUAL;
	m_CurrentWpIndex = -1;
	// Clear route
	m_RouteLength = 0;
	Waypoint temp;
	temp.ASL = 0;
	temp.Id = -1;
	temp.Position.East = 0;
	temp.Position.North = 0;
	for (int i = 0; i < MAX_ROUTE; i++)
	{
		m_Route[i] = temp;
	}

	printf("\nCreate UDP Channel TX=%s/%d, Rx=%s/%d\n", FLEET_CONTROOLER_IP, TxPort, PLATFORM_IP, RxPort);
	return true;
}

void GreenEye_Platform::Reset()
{
}

void GreenEye_Platform::Run()
{
	ReceiveMsg();
}

void GreenEye_Platform::ReceiveMsg()
{
	int BytesReceive;
	char RxBuffer[BUFFER_SIZE];

	// Receive message
	BytesReceive = m_RxChannel->timed_recv(RxBuffer, BUFFER_SIZE, 1000);

	if (BytesReceive > 0)
	{
		if (RxBuffer[0] == 'q')
		{
			m_SimState = SIMULATOR_STOP;
			printf ("Receive Quit command\n");
		}
		else
		{
			GEHeaderMsg *RcvMsg = (GEHeaderMsg*)&RxBuffer[0];
			//if (ValidateMsg(RcvMsg))
			{
				switch (RcvMsg->Opcode)
				{
					case GE_OP_CMD_SET_WP:
						RcvNewWP (&RxBuffer[0]);
						break;
					case GE_OP_SIM_CMD:
						RcvSimCommand (&RxBuffer[0]);
						break;
					case GE_OP_CMD_ROUTE:
						RcvRoute(&RxBuffer[0]);
						break;
					//case GE_OP_SET_MODE:
						//RcvMode(&RxBuffer[0]);
						//break;
				}
			}
		}

	}
}

bool GreenEye_Platform::ValidateMsg(GEHeaderMsg *RxMsg)
{
	return true;
}

bool GreenEye_Platform::RcvMode (char* a_buf)
{
	/*
	GEPlatformModeMsg *RcvMsg;

	RcvMsg = (GEPlatformModeMsg*)a_buf;
	PlatformMode mode = RcvMsg->Mode;

	m_FlightMode = mode.GetFlightMode();
	printf ("Set Flight mode = %d\n", m_FlightMode);
	*/
	return true;
}

bool GreenEye_Platform::RcvRoute (char* a_buf)
{
	printf ("New Route\n");
	ClearRoute();


	GERouteMsg	*RcvMsg;
	RcvMsg = (GERouteMsg*)a_buf;
	m_RouteLength = RcvMsg->NumWayPoints;

	for (int i=0; i < m_RouteLength; i++)
	{
		m_Route[i].ASL = RcvMsg->WP[i].ASL;
		m_Route[i].Id = RcvMsg->WP[i].Id;
		m_Route[i].Position = RcvMsg->WP[i].Position;
		m_Route[i].IsHoldState = RcvMsg->WP[i].IsHoldState;
		m_Route[i].TimeFromPlan = RcvMsg->WP[i].TimeFromPlan;
		m_Route[i].LegType = RcvMsg->WP[i].LegType;

			//m_Route[i] = RcvMsg->WP[i];

			printf("WP%d (%f,%f,%f), type=%d, wait=%d\n",
					i,
					RcvMsg->WP[i].Position.North,
					RcvMsg->WP[i].Position.East,
					RcvMsg->WP[i].ASL,
					RcvMsg->WP[i].LegType,
					RcvMsg->WP[i].TimeFromPlan );
	}

	m_FlightState = FLIGHT_FLY;
	m_State.m_MissionState = MISSION_STATE_STANDBY;

	m_MissionStartTime = ros::Time::now();

	GotoNextWaypoint();

	return true;
}


bool GreenEye_Platform::RcvNewWP (char* a_buf)
{
	GECommandMsg	*RcvMsg;
	RcvMsg = (GECommandMsg*)a_buf;
	Waypoint	RecvPoint;

	ClearRoute();
	//m_FlightMode = FLIGHT_MODE_AUTO;
	m_Route[0] = RcvMsg->WP;
	m_RouteLength = 1;

	m_FlightState = FLIGHT_FLY;
	m_State.m_MissionState = MISSION_STATE_STANDBY;
	m_MissionStartTime = ros::Time::now();
	GotoNextWaypoint();

	return true;
}

bool GreenEye_Platform::RcvSimCommand(char* a_buf)
{
	GESimCmdMsg			*RcvMsg;

	RcvMsg = (GESimCmdMsg*)a_buf;
	switch (RcvMsg->SimCommand)
	{
	case GE_SIM_EXIT:
		printf ("Receive QUIT command \n");
		m_State.m_MissionState = MISSION_STATE_HOME;
		StopCommand(true);
		break;

	case GE_SIM_RESET:
		printf ("Receive RESET command \n");
		ClearRoute();
		m_CurrentWP.Position = m_State.Position;
		m_CurrentWP.ASL = m_State.ASL;
		m_SimState = SIMULATOR_STOP;
		m_FlightState = FLIGHT_WAIT;
		m_State.m_MissionState = MISSION_STATE_HOME;

// ### Removed at 19/11/15 - Reset shouldn't move the platform
		//SetVelocityCommand();
		break;

	case GE_SIM_STOP:
		printf ("Receive STOP command \n");
		StopCommand(false);
		break;

	case GE_SIM_PLAY:
		printf ("Receive PLAY command \n");
		m_SimState = SIMULATOR_RUN;
		SetVelocityCommand();
		break;
	}

}

void GreenEye_Platform::SetState (const nav_msgs::Odometry& msg)
{
	static bool IsFirstTime = true;

	if (IsFirstTime)
	{
		IsFirstTime =false;
		m_LaunchPosition.Position.North = msg.pose.pose.position.x;
		m_LaunchPosition.Position.East = msg.pose.pose.position.y;
		m_LaunchPosition.ASL = msg.pose.pose.position.z;
		m_CurrentWP = m_LaunchPosition;
	}
	m_State.Position.East = msg.pose.pose.position.y;
	m_State.Position.North = msg.pose.pose.position.x;
	m_State.ASL = msg.pose.pose.position.z;
	m_State.Heading = msg.pose.pose.orientation.x;
	m_State.Roll = msg.pose.pose.orientation.w;
	m_State.Pitch = msg.pose.pose.orientation.z;
	m_State.SimTime = msg.header.stamp.sec;

	SetVelocityCommand();
	SendStateMsg();
}



void GreenEye_Platform::SendStateMsg()
{
	GEStatusMsg StateMsg;

	StateMsg.Header.StartBytes = GE_START_BYTES;
	StateMsg.Header.Dst = GE_GS_ADDRESS;
	StateMsg.Header.Src = m_Id;
	StateMsg.Header.MsgLength = sizeof(GEStatusMsg);
	StateMsg.Header.Opcode = GE_OP_STATUS;
	StateMsg.Header.MsgId = m_MsgTxCounter++;

	StateMsg.State = m_State;

	if (m_TxChannel != 0)
		m_TxChannel->send((char*)&StateMsg, sizeof(StateMsg));
}

void GreenEye_Platform::ClearRoute()
{
	m_RouteLength = 0;
	m_CurrentWpIndex = -1;
	m_State.m_MissionState = MISSION_STATE_HOME;
	// Reset current WP
	m_CurrentWP.Position = m_State.Position;
	m_CurrentWP.ASL = m_State.ASL;

}

void GreenEye_Platform::GotoNextWaypoint()
{
	if (m_CurrentWpIndex + 1 < m_RouteLength)
	{
		m_CurrentWpIndex++;

		Waypoint 	NextWp;

		NextWp = m_Route[m_CurrentWpIndex];
		if (NextWp.ASL == DONT_CHANGE_VALUE)
			NextWp.ASL = m_State.ASL;
			//NextWp.ASL = m_CurrentWP.ASL;
		if (NextWp.Position.East == DONT_CHANGE_VALUE)
			NextWp.Position.East = m_State.Position.East;//m_PlatformState.pose.position.y;
			//NextWp.Position.East = m_CurrentWP.Position.East;
		if (NextWp.Position.North == DONT_CHANGE_VALUE)
			NextWp.Position.North = m_State.Position.North;//m_PlatformState.pose.position.x;
			//NextWp.Position.North = m_CurrentWP.Position.North;

		m_CurrentWP = NextWp;
		//m_HoldTime.tv_sec = m_CurrentWP.TimeToWait;

		switch (m_CurrentWP.LegType)
		{
		case LEG_TYPE_TO:
			m_State.m_MissionState = MISSION_STATE_TO;
			break;
		case LEG_TYPE_MISSION:
			m_State.m_MissionState = MISSION_STATE_MISSION;
			break;
		case LEG_TYPE_FROM:
			m_State.m_MissionState = MISSION_STATE_FROM;
			break;
		}

		printf ("Continue to wp%d=(%f,%f,%f) and wait %d seconds\n",
				m_CurrentWpIndex,
				m_CurrentWP.Position.North,
				m_CurrentWP.Position.East,
				m_CurrentWP.ASL,
				m_CurrentWP.TimeFromPlan);
		m_FlightState = FLIGHT_FLY;

		SetVelocityCommand();
	}
	else
	{
		m_FlightState = FLIGHT_END;
		m_State.m_MissionState = MISSION_STATE_RECOVER;
	}
}


void GreenEye_Platform::SetVelocityCommand()
{
	double		deltaX,
				deltaY,
				deltaZ;

	double MaxVel = 3;//0.4;
	double MaxVelZ = 1;
	double CloseDistToWP = 0.4;
	geometry_msgs::Twist pubVel;
	deltaX = m_CurrentWP.Position.North - m_State.Position.North;// m_PlatformState.pose.position.x;
	deltaY = m_CurrentWP.Position.East - m_State.Position.East;//- m_PlatformState.pose.position.y;
	deltaZ = m_CurrentWP.ASL - m_State.ASL;//- m_PlatformState.pose.position.z;
	double dist = sqrt((deltaX * deltaX) + (deltaY * deltaY));

	pubVel.linear.x = pubVel.linear.y = pubVel.linear.z = 0;

	// ### Added at 18/11/15 - no need to move platform if at home/end
	if (m_FlightState == FLIGHT_WAIT || m_FlightState == FLIGHT_END)
	{
		m_VelPublish.publish(pubVel);
	}
	else
	{
		// reach near the destination?
		if (dist <= CloseDistToWP && abs(deltaZ) <= CloseDistToWP)
		{
			switch (m_FlightState)
			{
				case FLIGHT_HOLD:
				{
					// ### Update 5/11/15 - Hold time is relative to re-plan time
					ros::Duration DiffTime = ros::Time::now() - m_MissionStartTime;
					if (DiffTime.sec >= m_CurrentWP.TimeFromPlan)
						GotoNextWaypoint();
					break;
				}

				case FLIGHT_FLY:
				{
					if (m_CurrentWP.IsHoldState)
					{
						m_FlightState = FLIGHT_HOLD;
						m_VelPublish.publish(pubVel);
					}
					else
						GotoNextWaypoint();
					break;
				}

				default:
					printf ("Mission state error\n");
					break;
			}
		}
		else // Set velocity to correct direction
		{
			double angle;
			double CurVel;
			// Is platform reach near its destination?
			if (dist > CloseDistToWP)
			{
				if (deltaX == 0)
				{
					if (deltaY > 0)
						angle = 3.1416/2.0;
					else
						angle = -3.1416/2.0;
				}
				else
					angle = atan2(deltaY, deltaX);
				if (angle < 0)
					angle = 2 * 3.1416 + angle;

				CurVel = min (dist, MaxVel);
				pubVel.linear.x = CurVel* cos (angle);
				pubVel.linear.y = CurVel* sin (angle);
				printf ("(dist,ang) = (%.2f, %.2f) (velx,vely)= (%.2f, %.2f) \n", dist, angle, pubVel.linear.x, pubVel.linear.y);
			}
			else
			{
				pubVel.linear.x = pubVel.linear.y = 0;
				CurVel = min(abs(deltaZ), MaxVelZ);
				if (deltaZ > CloseDistToWP)
					pubVel.linear.z = CurVel;
				if (deltaZ < -CloseDistToWP)
					pubVel.linear.z = -CurVel;
			}

			m_VelPublish.publish(pubVel);
		} // end of velocity update
	}

	m_State.Velocity.x = pubVel.linear.x;
	m_State.Velocity.y = pubVel.linear.y;
	m_State.Velocity.z = pubVel.linear.z;
	//m_PeriodTime = CurrentTime;

}

void GreenEye_Platform::StopCommand(bool a_Quit)
{
	//geometry_msgs::Twist pubVel;
	//pubVel.linear.x = pubVel.linear.y = pubVel.linear.z = 0;
	//m_VelPublish.publish(pubVel);
	printf ("Stop command active\n");
	m_SimState = SIMULATOR_STOP;
	SetVelocityCommand();
}

void GreenEye_Platform::ResumeCommand()
{
	m_SimState = SIMULATOR_RUN;
	SetVelocityCommand();
}


