#pragma once


//#include "gazebo-2.2/gazebo/gazebo.hh"
#include <gazebo/gazebo.hh>
#include "FleetController_constants.h"
#include "../../Infra/UDPChannel.h"
#include <pthread.h>

#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/SpawnModel.h"


#include <std_srvs/EmptyRequest.h>

class GazeboServer
{
private:
	udp_server 		*m_RxChannel;
	udp_client		*m_TxChannel;
	pthread_t 	m_hThread;

	int			m_LastRxMsgNum;
	int			m_LastTxMsgNum;

	int 		m_FleetSize;

	//ros::ServiceClient m_StateClient;
	ros::NodeHandle m_NodeHandler;

	void RcvThread();
	bool RcvSimCommand(char* a_buf);
	

public:
	GazeboServer(ros::NodeHandle a_nodeHandler);
	~GazeboServer();

	bool SetCommunication ();

	void SetParameters(int a_FleetSize);

	void ResetSimulator();

	void TestFunction ();
/*

	bool SendWayPoint(Waypoint wp);
	bool SendStatusReqMsg ();
	bool SendSimCommandMsg (int SimCmd);

	bool SendRoute (Waypoint* a_route, int a_Length);


	//bool SendMode (PlatformMode mode);
*/

	static void* static_thread (void *args)
	{
		static_cast<GazeboServer*>(args)->RcvThread();
		return 0;
	}



	

};

