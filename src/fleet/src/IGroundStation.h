#pragma once

#include "../../Common/PlatformIF.h"


class IGroundStation
{
private:
	void Init();

protected:
	int				m_Id;
	IPlatform		*m_pPlatform;

	IPlatform_State m_LastState;

public:
	IGroundStation(int GS_ID);
	virtual ~IGroundStation();

	void SetPlatform (IPlatform	*pPlatform);

	void Reset();

	virtual bool SendWayPoint(Waypoint wp) = 0;
	virtual bool SendStatusReqMsg () = 0;
	virtual bool SendSimCommandMsg (int SimCmd) = 0;
	virtual bool SendRoute (Waypoint* a_route, int a_Length) = 0;
//	virtual bool SendMode (PlatformMode mode) = 0;

	IPlatform_State GetState() { return m_LastState;};

protected:
	virtual bool SetCommunication() = 0;

};

