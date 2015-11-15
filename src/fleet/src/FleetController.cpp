#include "FleetController.h"
#include "GreenEye_GS.h"


FleetController::FleetController()
{
}


FleetController::~FleetController()
{
	for (map<int, IGroundStation*>::iterator it = m_GroundStations.begin(); it != m_GroundStations.end(); ++it)
	{
		delete (it->second);
	}
	m_GroundStations.erase(m_GroundStations.begin(), m_GroundStations.end());
}

bool FleetController::CreateFleet(int a_GroupSize)
{
	IPlatform* platform;
	for (int id = 0; id < a_GroupSize; id++)
	{
		platform = new IPlatform(id);
		CreateComm (platform);
	}
	return true;
}

bool FleetController::AddPlatform(int id)
{
// #Temp - should read from AppServer and create GS for each platform	
	// platforms = AppServer->GetPlatforms();
	// foreach (platform)
	// CreateComm (platform)

	IPlatform* platform = new IPlatform(id);
	CreateComm (platform);
	
	return true;
}

bool FleetController::Reset()
{

	return true;
}

bool FleetController::CreateComm (IPlatform *platform)
{
	int			PlatformId;
					
	PlatformId = platform->GetId();
	// Check if there is already GS for request platform
	if (m_GroundStations.count(PlatformId) > 0)
	{		
		return false;
	}

	m_GroundStations[PlatformId] = new GreenEye_GS(PlatformId);
	m_GroundStations[PlatformId]->SetPlatform (platform);


	return true;
}

bool FleetController::SendCmdTo (IPlatform* platform, Waypoint wp)
{
	int					PlatformId;
						
	PlatformId = platform->GetId();

	return SendCmdTo(PlatformId, wp);				
}

bool FleetController::SendCmdTo (int PlatformId, Waypoint wp)
{
	IGroundStation		*GS;

	// Check if there is a GS for request platform
	if (m_GroundStations.count(PlatformId) == 0)
	{
		return false;
	}

	GS = m_GroundStations[PlatformId];
	GS->SendWayPoint(wp);
	return true;
}

bool FleetController::SendRouteTo (int PlatformId, Waypoint* a_route, int a_Length)
{
	IGroundStation		*GS;
	if (m_GroundStations.count(PlatformId) == 0)
	{
		return false;
	}

	GS = m_GroundStations[PlatformId];
	GS->SendRoute(a_route, a_Length);
	return true;
}

bool FleetController::SendSimulatorCommand(int a_SimCommand)
{
	for (map<int, IGroundStation*>::iterator it = m_GroundStations.begin(); it != m_GroundStations.end(); ++it)
	{
		it->second->SendSimCommandMsg(a_SimCommand);
	}

	return true;
}

/*
bool FleetController::SetMode (int PlatformId, PlatformMode mode)
{
	IGroundStation		*GS;
	if (m_GroundStations.count(PlatformId) == 0)
	{
		return false;
	}

	GS = m_GroundStations[PlatformId];
	GS->SendMode(mode);
	return true;
}
*/


bool FleetController::ReceiveStatus ()
{
	for (map<int, IGroundStation*>::iterator it = m_GroundStations.begin(); it != m_GroundStations.end(); ++it)
	{
		it->second->GetState();
	}
	return true;
}

bool FleetController::GetStatusFrom (int PlatformId, IPlatform_State& pState)
{
	IGroundStation		*GS;
	IPlatform_State		state;
	if (m_GroundStations.count(PlatformId) == 0)
	{
		return false;
	}

	GS = m_GroundStations[PlatformId];
	pState = GS->GetState();
	//pState = state;

	return true;
}
