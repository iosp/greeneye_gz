#pragma once

#include <vector>
//#include "Common.h"
#include "TopologyIF.h"
#include "MissionIF.h"

using namespace std;

typedef enum
{
	FLIGHT_MODE_MANUAL = 0,
	FLIGHT_MODE_AUTO
} FLIGHT_MODE;

typedef enum
{
	MISSION_STATE_HOME = 0,
	MISSION_STATE_STANDBY,
	MISSION_STATE_TO,
	MISSION_STATE_MISSION,
	MISSION_STATE_FROM,
	MISSION_STATE_RECOVER
} MISSION_STATE;

class ISensor
{
public:
	int			Id;
	int			Type;
	int			State;
	Point2d		Position;
	double		HFov;
	double		VFov;

public:
	ISensor(){};
	~ISensor(){};

};

class IPlatform_Config
{
private:
	int			m_Type;
	double		m_Energy;
	double		m_MinVelocity;
	double		m_MaxVelocity;
	double		m_EnergyCost;

public:
	IPlatform_Config(){};
	~IPlatform_Config(){};
};

class IPlatform_State
{

public:
	int			Type;
	Point2d		Position;
	double		ASL;
	double		Heading;
	double		Roll;
	double		Pitch;
	double3d	Velocity;
	int			DeployState;		// unused
	MISSION_STATE m_MissionState;
	int			FailureState;
	double		Energy;
	int			SimTime;

public:
	IPlatform_State(){};
	~IPlatform_State(){};

	//double GetASL() { return ASL;};
	//Point2d GetPosition() { return Position; };
	//void SetPosition(Point2d point) { Position = point;};
	//void SetASL (double asl) {ASL = asl;};

};

class IPlatform
{
private:
	int					m_Id;
	int					Type;
	IPlatform_State*	State;
	IPlatform_Config*	Config;
	vector<ISensor*>	Sensors;
	vector<Waypoint>	Route;
	// ICommunication*	Comm		// TBD
public:
	IPlatform(int Id) { m_Id = Id; };

	~IPlatform(){};

	int GetId () { return m_Id; }

};

class PlatformMode
{
private:
	FLIGHT_MODE		m_FlightMode;
public:
	PlatformMode() {m_FlightMode = FLIGHT_MODE_MANUAL;};
	void SetFlightMode (FLIGHT_MODE mode) {m_FlightMode = mode;};
	FLIGHT_MODE GetFlightMode () {return m_FlightMode;};
};

//class QuadRotor_Config : IPlatform_Config
//{
//private:	
//	double		MaxZVelocity;
//public:
//	QuadRotor_Config(){};
//	~QuadRotor_Config(){};
//}


