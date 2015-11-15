#pragma once

#include <vector.h>
#include "MissionIF.h"

using namespace std;

typedef struct
{
	// WindTable
	// Weather
	// TOD
	// Global Map
} EnviromentConfig;

typedef Simulator
{
	vector<int>			MissionsIds;
	EnviromentConfig	Enviroment;
	vector<Area>		RestrictedAreas;
	int					SimCommand;
	int					RecordState;
}
