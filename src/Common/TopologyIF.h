#pragma once

#include <vector>



using namespace std;

typedef struct
{
	double		x;
	double		y;
	double		z;

} double3d;

typedef struct
{
	double	North;
	double	East;
	int		Zone;
	int		Hemisphere;
} Point2d;

typedef struct
{
	Point2d		UpperLeft;
	Point2d		BottomRight;
} Rect;

typedef struct
{
	int			Id;
	Point2d		Start;
	Point2d		End;
	double		MinHeight;
	double		MaxHeight;
	double		Direction;
} Gateway;

typedef struct
{
	int			Id;
	vector<Point2d>	Polygon;
	vector<Gateway> GwInList;
	vector<Gateway> GwOutList;
	vector<Gateway> GwEmergencyList;
} Area;

typedef struct
{
	int			Id;
	Point2d		Position;
	double		ASL;
	int			State;
	int			TimeToPrepare;
} RecoveryPoint;




