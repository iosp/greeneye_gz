#include "GazeboServer.h"

#include "../../Common/Common.h"
#include "ros/service.h"
#include "ros/service_client.h"
#include <math.h>


GazeboServer::GazeboServer(ros::NodeHandle a_nodeHandler)
{
	m_RxChannel = 0;
	m_TxChannel = 0;
	m_LastRxMsgNum = -1;
	m_LastTxMsgNum = -1;
	m_FleetSize = 1;

	m_NodeHandler = a_nodeHandler;
}

GazeboServer::~GazeboServer()
{
	printf ("Disconnect server\n");
	if (m_RxChannel != 0)
		delete (m_RxChannel);
	if (m_TxChannel != 0)
	{				
		delete (m_TxChannel);
	}
	m_RxChannel = 0;
	m_TxChannel = 0;
	m_hThread = 0;
}

void GazeboServer::SetParameters (int a_FleetSize)
{
	m_FleetSize = a_FleetSize;
}

bool GazeboServer::SetCommunication ()
{
	int		TxPort,
			RxPort;
	int 	result;

	if (m_RxChannel != 0)
		delete(m_RxChannel);
	if (m_TxChannel != 0)
	{
		delete (m_TxChannel);
	}

	TxPort = FLEET_SERVER_TX;
	RxPort = FLEET_SERVER_RX;

	m_RxChannel  = new udp_server(LOCAL_IP, RxPort);
	m_TxChannel = new udp_client(FLEET_CONTROOLER_IP, TxPort);

	printf("\nCreate UDP Channel TX=%s/%d, Rx=%s/%d\n", FLEET_CONTROOLER_IP, TxPort, LOCAL_IP, RxPort);

	m_RxChannel->Connect(false);

	result = pthread_create(&m_hThread, NULL, GazeboServer::static_thread, this);
	if (result != 0)
	{
		printf ("pthread_create failed for gazebo server\n");
		return false;
	}

	m_LastRxMsgNum = -1;
	m_LastTxMsgNum = -1;

	return true;
}


//////////////////////////////////////////////////////////////
//															//
//					Receive Functions						//
//															//
//////////////////////////////////////////////////////////////
void GazeboServer::RcvThread()
{
	int			BytesRecv;
	char		RxBuffer [BUFFER_SIZE];
		
	printf ("Gazebo Server Start listen\n");

	while (m_hThread != 0)
	{
		if (m_RxChannel != 0)
			BytesRecv = m_RxChannel->timed_recv(RxBuffer, BUFFER_SIZE, 10);
		if (BytesRecv > 0)
		{		
			GEHeaderMsg *RcvMsg = (GEHeaderMsg*)&RxBuffer[0];
			//if (ValidateMsg(RcvMsg))
			{
				switch (RcvMsg->Opcode)
				{
						break;
					case GE_OP_SIM_CMD:
						RcvSimCommand (&RxBuffer[0]);
						break;
				}
			}
		}
	}

	printf ("pthread_exit for gazebo server\n");
	pthread_exit(0);

}

bool GazeboServer::RcvSimCommand(char* a_buf)
{
	GESimCmdMsg			*RcvMsg;

	RcvMsg = (GESimCmdMsg*)a_buf;
	switch (RcvMsg->SimCommand)
	{
	case GE_SIM_EXIT:
		printf ("Receive QUIT command \n");
		break;

	case GE_SIM_RESET:
		printf ("Receive RESET command \n");
		ResetSimulator();
		break;

	case GE_SIM_STOP:
		printf ("Receive STOP command \n");

		break;

	case GE_SIM_PLAY:
		printf ("Receive PLAY command \n");
		break;
	}
}

void GazeboServer::ResetSimulator()
{
	gazebo_msgs::SetModelState setmodelstate;
	gazebo_msgs::ModelState modelstate;
	char platform_name [10];

	int NumOfRows = 1 + (int)sqrt(m_FleetSize);
	ros::ServiceClient client = m_NodeHandler.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

	int OffsetX, OffsetY;
	OffsetX = 0;//3554244;
	OffsetY = 0;//669953;
	for (int platformId = 1; platformId <= m_FleetSize; platformId++)
	{
		sprintf(platform_name, "uav%d", platformId);
		modelstate.model_name = platform_name;
		modelstate.reference_frame = "world";

		geometry_msgs::Twist model_twist;
		model_twist.linear.x = 1.0;
		model_twist.linear.y = 1.0;
		model_twist.linear.z = 0.0;
		model_twist.angular.x = 0.25;
		model_twist.angular.y = 0.0;
		model_twist.angular.z = 0.0;
		modelstate.pose.position.x =  OffsetX + (platformId-1) / NumOfRows;
		modelstate.pose.position.y = OffsetY + (platformId-1) % NumOfRows;
		modelstate.pose.position.z = 0.5;
		modelstate.twist = model_twist;
		setmodelstate.request.model_state = modelstate;
		client.call(setmodelstate);
	}
}

void GazeboServer::TestFunction ()
{
	//gazebo::runWorld()()stop();

	//std_srvs::EmptyRequest req;

	std_msgs::Header req();
	std_msgs::Header res();

	ros::service::call("/uav1/shutdown", req, res);
	//ros::ServiceClient client = m_NodeHandler.serviceClient("/uav1/shutdown");


	//std_srvs::EmptyRequest req;

	//client.call(req);

	//gazebo::physics::World world;
	//world.SetPaused(true);

	/*
	//gazebo_msgs::SpawnModel
	gazebo_msgs::SpawnModel spawn;
	//gazebo_msgs::SpawnModelRequest request;

	 ros::ServiceClient client = m_NodeHandler.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_urdf_model");

	spawn.request.initial_pose.position.x = 5;
	spawn.request.initial_pose.position.y = 1;
	spawn.request.initial_pose.position.z = 0;


	std::string urdf_filename = "quadrotor.gazebo.xacro";

	TiXmlDocument xml_in(urdf_filename);
	xml_in.LoadFile();
	 std::ostringstream stream;
	 stream << xml_in;
	 spawn.request.model_xml = stream.str(); // load xml file



	client.call(spawn);

*/

}


