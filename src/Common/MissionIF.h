#pragma once

#include <vector>
#include "TopologyIF.h"


using namespace std;

/*
class IAction
{
private:
	int		Type;

public:
	IAction(void) {};
	~IAction(void) {};

};
*/

typedef enum
{
	LEG_TYPE_TO = 0,
	LEG_TYPE_MISSION,
	LEG_TYPE_FROM
} LEG_TYPE;

typedef struct
{
	int					Id;
	Point2d				Position;
//	vector<IAction*>	Actions;
	double				ASL;
	bool				IsHoldState;
	int					TimeFromPlan;
	LEG_TYPE			LegType;

} Waypoint;

typedef struct
{
	int			MaxActivePlatforms;
	//DateTime	StartTime;
	//DateTime	EndTime;
	double		StayHeight;
} IMissionConfig;



typedef struct
{
	int				Id;
	int				Type;
	Area			MissionArea;
	vector<int>		RecoverySitesIds;
	vector<int>		LauncherIds;
	IMissionConfig	Config;
} Mission;


