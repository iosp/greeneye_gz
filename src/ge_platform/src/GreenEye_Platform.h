/*
 * ge_com_node.h
 *
 *  Created on: Jul 19, 2015
 *      Author: robil
 */

#pragma once

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"


#include "GreenEye_Platform_Constants.h"
#include "../../Common/Common.h"
#include "../../Common/PlatformIF.h"
#include "../../Infra/UDPChannel.h"


class GreenEye_Platform
{
	/*typedef enum
	{

	} FLIGHT_MODE;
	*/

	typedef enum
	{
		FLIGHT_WAIT = 0,
		FLIGHT_FLY,
		FLIGHT_HOLD,
		FLIGHT_END
	} FLIGHT_STATE;

	typedef enum
	{
		SIMULATOR_STOP = 0,
		SIMULATOR_RUN,
		//SIMULATOR_PAUSE ??
	} SIMULATOR_STATE;

private:
	int				m_Id;

	udp_server 		*m_RxChannel;
	udp_client		*m_TxChannel;
	int 			m_MsgTxCounter;

	Waypoint		m_LaunchPosition;
	Waypoint		m_Route [MAX_ROUTE];
	int				m_CurrentWpIndex;
	int 			m_RouteLength;
	Waypoint		m_CurrentWP;

	IPlatform_State	m_State;

	//geometry_msgs::PoseStamped	m_PlatformState;

	//MISSION_STATE			m_MissionState;
	FLIGHT_STATE			m_FlightState;
	SIMULATOR_STATE			m_SimState;
	//FLIGHT_MODE					m_FlightMode;

	//struct timeval m_HoldTime;		// Time to hold in the last position
	//struct timeval m_PeriodTime;	// Elapsed time between two calls for SetVelocity

	int m_StartTime;

	ros::Time m_MissionStartTime;


	bool Init();
	void ReceiveMsg();
	bool ValidateMsg (GEHeaderMsg *RxMsg);

	void RcvThread() {};

	bool RcvNewWP (char* a_buf);
	bool RcvMode (char* a_buf);
	bool RcvRoute (char* a_buf);
	bool RcvSimCommand (char* a_buf);

	void SendStateMsg ();

	void ClearRoute();
	void GotoNextWaypoint ();
	void SetVelocityCommand();



public:
	GreenEye_Platform(int a_Id);
	~GreenEye_Platform();

	void Reset();
	void Run();

	//void SetState(const geometry_msgs::PoseStamped& msg);
	void SetState (const nav_msgs::Odometry& msg);

	IPlatform_State GetState () {return m_State;};

	//geometry_msgs::PoseStamped GetState (){return m_PlatformState;};

	bool GetSimulatorState () {return m_SimState;};

	void ResumeCommand();

	void StopCommand(bool a_IsQuit);

	ros::Publisher 	m_VelPublish;


};


