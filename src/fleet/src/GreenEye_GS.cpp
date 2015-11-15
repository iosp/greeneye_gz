#include "GreenEye_GS.h"
#include "../../Common/GEMessages.h"

#include <stdio.h>
#include <string.h>

GreenEye_GS::GreenEye_GS(int m_Id) : IGroundStation(m_Id)
{
	Reset();
}

GreenEye_GS::GreenEye_GS(int m_Id, IPlatform *pPlatform) : IGroundStation(m_Id)
{
	Reset();
}

bool GreenEye_GS::Reset()
{
	m_RxChannel = 0;
	m_TxChannel = 0;
	m_LastRxMsgNum = -1;
	m_LastTxMsgNum = -1;

	return true;
}

GreenEye_GS::~GreenEye_GS()
{
	printf ("Disconnect Platform %d\n", m_pPlatform->GetId());
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

bool GreenEye_GS::SetCommunication ()
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

	TxPort = BASE_PORT_TX + m_pPlatform->GetId();
	RxPort = BASE_PORT_RX + m_pPlatform->GetId();

	m_RxChannel  = new udp_server(LOCAL_IP, RxPort);
	m_TxChannel = new udp_client(FLEET_CONTROOLER_IP, TxPort);

	printf ("Connect Platform %d with TxPort=%d RxPort=%d\n", m_pPlatform->GetId(), TxPort, RxPort);

	m_RxChannel->Connect(false);

	result = pthread_create(&m_hThread, NULL, GreenEye_GS::static_thread, this);
	if (result != 0)
	{
		printf ("pthread_create failed for platform%d\n", m_Id);
		return false;
	}

	m_LastRxMsgNum = -1;
	m_LastTxMsgNum = -1;

	return true;
}

//////////////////////////////////////////////////////////////
//															//
//					Send Functions							//
//															//
//////////////////////////////////////////////////////////////
GEHeaderMsg GreenEye_GS::CreateHeader (int Opcode, int MsgLength)
{
	GEHeaderMsg header;
	header.StartBytes = GE_START_BYTES;
	header.MsgId = m_LastTxMsgNum++;
	header.Src = -1;
	header.Dst = m_pPlatform->GetId();
	header.MsgLength = MsgLength;
	header.Opcode = Opcode;

	return header;
}

bool GreenEye_GS::SendWayPoint(Waypoint wp)
{
	GECommandMsg msg;
	msg.Header.StartBytes = GE_START_BYTES;
	msg.Header.MsgId = m_LastTxMsgNum++;	
	msg.Header.Src = -1;
	msg.Header.Dst = m_pPlatform->GetId();
	msg.Header.MsgLength = sizeof(GECommandMsg);
	msg.Header.Opcode = GE_OP_CMD_SET_WP;

	msg.WP = wp;	

	m_TxChannel->send((char*)&msg, sizeof(GECommandMsg));

	return true;
}

bool GreenEye_GS::SendRoute (Waypoint* a_route, int a_Length)
{
	GERouteMsg msg;
	msg.Header = CreateHeader(GE_OP_CMD_ROUTE, sizeof(msg));
	msg.NumWayPoints = a_Length;
	memcpy(&msg.WP[0], a_route, a_Length * sizeof(Waypoint));
	//msg.WP = a_route;

	m_TxChannel->send((char*)&msg, sizeof(GERouteMsg));
	return true;
}

bool GreenEye_GS::SendStatusReqMsg ()
{
	GEReqStatusMsg msg;

	msg.Header.StartBytes = GE_START_BYTES;
	msg.Header.MsgId = m_LastTxMsgNum++;	
	msg.Header.Src = -1;
	msg.Header.Dst = m_pPlatform->GetId();
	msg.Header.MsgLength = sizeof(GEReqStatusMsg);
	msg.Header.Opcode = GE_OP_REQ_STATUS;

	m_TxChannel->send((char*)&msg, sizeof(GEReqStatusMsg));

	return true;
}

bool GreenEye_GS::SendSimCommandMsg (int SimCmd)
{
	GESimCmdMsg msg;
	msg.Header.StartBytes = GE_START_BYTES;
	msg.Header.MsgId = m_LastTxMsgNum++;
	msg.Header.Src = -1;
	msg.Header.Dst = m_pPlatform->GetId();
	msg.Header.MsgLength = sizeof(GESimCmdMsg);
	msg.Header.Opcode = GE_OP_SIM_CMD;

	msg.SimCommand = (GE_Sim_Opcodes)SimCmd;

	m_TxChannel->send((char*)&msg, sizeof(GESimCmdMsg));

	return true;
}

/*
bool GreenEye_GS::SendMode (PlatformMode mode)
{
	GEPlatformModeMsg msg;
	msg.Header.StartBytes = GE_START_BYTES;
	msg.Header.MsgId = m_LastTxMsgNum++;
	msg.Header.Src = -1;
	msg.Header.Dst = m_pPlatform->GetId();
	msg.Header.MsgLength = sizeof(GESimCmdMsg);
	msg.Header.Opcode = GE_OP_SET_MODE;

	msg.Mode = mode;

	m_TxChannel->send((char*)&msg, sizeof(GEPlatformModeMsg));

	return true;
}
*/

//////////////////////////////////////////////////////////////
//															//
//					Receive Functions						//
//															//
//////////////////////////////////////////////////////////////
void GreenEye_GS::RcvThread()
{
	int			BytesRecv;
	char		buf [BUFFER_SIZE];
		
	printf ("Thread Start for platform %d\n", m_Id);

	while (m_hThread != 0)
	{
		if (m_RxChannel != 0)
			BytesRecv = m_RxChannel->timed_recv(buf, BUFFER_SIZE, 10);
		if (BytesRecv > 0)
		{		
			GEHeaderMsg* msg = (GEHeaderMsg*) &buf[0];
			if (ValidateMsg(msg))
			{
				m_LastRxMsgNum = msg->MsgId;
				// Decode message
				switch (msg->Opcode)
				{
				case GE_OP_ACK:
					RcvAckMsg (&buf[0]);
					break;
				case GE_OP_STATUS:
					RcvStatus (&buf[0]);
					break;
				}

			}

		}
	}

	printf ("pthread_exit for platform%d\n", m_Id);
	pthread_exit(0);


}


bool GreenEye_GS::ValidateMsg (GEHeaderMsg* RxMsg)
{
	int ErrorCode = -1;

	if (RxMsg->StartBytes != GE_START_BYTES)	
		ErrorCode = 1;	
	else
		if (RxMsg->MsgLength > BUFFER_SIZE)
			ErrorCode = 2;		
		else
			if (RxMsg->MsgId < m_LastRxMsgNum)			
				ErrorCode = 3;			
			else 
				if (RxMsg->Dst != GS_ADDRESS)				
					ErrorCode = 4;				
				else
					if (RxMsg->Src != m_pPlatform->GetId())				
						ErrorCode = 5;				


	switch (ErrorCode)
	{
	case -1:
		return true;
	case 1:
		printf ("RcvMessage(), Start bytes error");
		break;
	case 2:
		printf ("RcvMessage(), Message length error");
		break;
	case 3:
		printf ("RcvMessage(), Receive older message");
		break;
	case 4:
		printf ("RcvMessage(), Invalid Destination");
		break;
	case 5:
		printf ("RcvMessage(), Invalid Source address");
		break;

	}

	return false;
}

void GreenEye_GS::RcvAckMsg (char* buf)
{

}

void GreenEye_GS::RcvStatus (char* buf)
{
	GEStatusMsg*	msg;
	msg = (GEStatusMsg*) buf;	

	m_LastState = msg->State;
/*
	Point2d Position = m_LastState.GetPosition();
	if (m_Id == 0)
		printf ("Id=%d (%f,%f,%f)\n",
			m_Id,
			Position.North,
			Position.East,
			m_LastState.GetASL());
			*/

}

