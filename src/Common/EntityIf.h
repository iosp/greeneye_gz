#pragma once

#include <vector>
#include "TopologyIF.h"

using namespace std;


typedef struct
{
	int			Id;
	Point2d		Position;
	double		ASL;
	double		Direction;
	int			TimeToLoad;
	vector<int>	PlatformsIds;
} Launcher;


