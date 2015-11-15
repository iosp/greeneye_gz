#include "IGroundStation.h"



IGroundStation::IGroundStation(int GS_ID)
{
	Reset();
	m_Id = GS_ID;
}


IGroundStation::~IGroundStation()
{
	int d = 5;
}

void IGroundStation::Reset()
{
	m_pPlatform = 0;
	m_Id = 0;
}

void IGroundStation::SetPlatform (IPlatform	*pPlatform)
{
	// Check if replace platform
	if (m_pPlatform == 0 || m_pPlatform->GetId() != pPlatform->GetId())
	{
		m_pPlatform = pPlatform;
		SetCommunication();
	}
}

