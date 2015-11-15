#pragma once

#include <map>
#include "IGroundStation.h"
#include "../../Common/PlatformIF.h"



class FleetController
{
private:
	std::map<int, IGroundStation*> m_GroundStations;
	
	bool CreateComm (IPlatform* platform);	

	bool SendCmdTo (IPlatform* platform, Waypoint wp);

	bool ReceiveStatus ();


public:
	FleetController(void);
	~FleetController(void);	

	bool Reset();

	bool CreateFleet(int a_GroupSize);
	bool AddPlatform(int id);

	bool SendCmdTo (int PlatformId, Waypoint wp);
	bool SendRouteTo (int PlatformId, Waypoint* a_route, int a_Length);

	bool SendSimulatorCommand(int a_SimCommand);
	
	//bool SetMode (int PlatformId, PlatformMode mode);

	bool GetStatusFrom (int PlatformId, IPlatform_State &pState);



};

