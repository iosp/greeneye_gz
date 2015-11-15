#pragma once


#include "IGroundStation.h"

#include "../../Common/PlatformIF.h"
#include "../../Common/GEMessages.h"

#include "FleetController_constants.h"
#include "../../Infra/UDPChannel.h"
#include <pthread.h>


/*

#define		UDP_TX_BASE_PORT		5000		// platform port = BASE_PORT + PLATFORM_ID
#define		UDP_RX_BASE_PORT		6000
#define		PLATFORM_SERVER_IP		"192.168.0.8"

#define		BUFFER_LENGTH			1024
#define		GE_START_BYTES			0x5050
*/
#define		GS_ADDRESS				-1


class GreenEye_GS : public IGroundStation
{
private:
	udp_server 		*m_RxChannel;
	udp_client		*m_TxChannel;

	pthread_t 	m_hThread;

	int			m_LastRxMsgNum;
	int			m_LastTxMsgNum;

	

	bool ValidateMsg (GEHeaderMsg* RxMsg);
	void RcvAckMsg (char* buf);
	void RcvStatus (char* buf);
	void RcvThread();

	GEHeaderMsg CreateHeader (int Opcode, int MsgLength);

public:
	GreenEye_GS(int m_Id);
	GreenEye_GS(int m_Id, IPlatform *pPlatform);
	~GreenEye_GS();

	bool Reset();
	bool SendWayPoint(Waypoint wp);
	bool SendStatusReqMsg ();
	bool SendSimCommandMsg (int SimCmd);

	bool SendRoute (Waypoint* a_route, int a_Length);


	//bool SendMode (PlatformMode mode);


	static void* static_thread (void *args)
	{
		static_cast<GreenEye_GS*>(args)->RcvThread();
		return 0;
	}

protected:
	bool SetCommunication ();
	

};

