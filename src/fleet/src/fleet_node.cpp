/*
 * Green Eye Communication Node
 * 1. Receive platformId from command line
 * 2. Create UDP Channel
 * 3. Control velocity
 * 4. Register to position topic
 *
 */
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <list>
#include <math.h>


#include "FleetController_constants.h"
#include "../../Common/MissionIF.h"
#include "../../Common/GEMessages.h"
//#include "../../Common/PlatformIF.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "FleetController.h"

using namespace std;

struct termios stdio;
struct termios old_stdio;

void SetKeyboardInput()
{
	tcgetattr(STDOUT_FILENO,&old_stdio);
	memset(&stdio,0,sizeof(stdio));

	stdio.c_iflag=0;
	stdio.c_oflag=0;
	stdio.c_cflag=0;
	stdio.c_lflag=ISIG;
	stdio.c_cc[VMIN]=1;
	stdio.c_cc[VTIME]=0;
	stdio.c_cc[VINTR]=1;

	tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
	tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);

	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking

}

Point2d GetUniformOffset (int PlatformId, int GroupSize)
{
	int n = sqrt(GroupSize);
	Point2d offset;
	offset.East = (int)(PlatformId/n);
	offset.North = (int)(PlatformId % n);

	return offset;
}

Point2d GetLineOffset (int PlatformId, int GroupSize)
{
	Point2d offset;
	offset.East = PlatformId;
	offset.North = 0;

	return offset;
}

Waypoint* ConvertListToArray (list<Waypoint> route)
{
	// convert list to array
	int length = route.size();
	Waypoint* RouteArr = new Waypoint[length];

	for (int i=0; i < length; i++)
	{
		RouteArr[i] = route.front();
		route.pop_front();
	}
	return RouteArr;
}

list<Waypoint> CreateDeployRoute(int PlatformId, int GroupSize)
{
	list<Waypoint> route;

	Waypoint 	wp;
	Point2d 	offset;

	int DelayOnLaunch = 12;
	// WP0 - Delay between launches
	wp.Id = 0;
	wp.Position.East = DONT_CHANGE_VALUE;
	wp.Position.North = DONT_CHANGE_VALUE;
	wp.ASL = DONT_CHANGE_VALUE;
	wp.IsHoldState = true;
	wp.TimeFromPlan = PlatformId * DelayOnLaunch;
	route.push_back(wp);
	// WP1 - Raise to traffic height
	wp.Id = 1;
	wp.Position.North = 0;
	wp.Position.East = 0;
	wp.ASL = 1;
	wp.IsHoldState = false;
	route.push_back(wp);
	// WP1A - Raise to traffic height
		wp.Id = 1;
		wp.Position.North = 0;
		wp.Position.East = 0;
		wp.ASL = 2;
		wp.IsHoldState = false;
		route.push_back(wp);
	// WP2 - go to mission area (enter gateway)
	wp.Id = 2;
	wp.Position.North = 5;
	wp.Position.East = 0;
	wp.ASL = 2;
	route.push_back(wp);
	// WP3 - reach to specfic deploy point
	offset = GetUniformOffset(PlatformId, GroupSize);
	wp.Id = 3;
	wp.Position.North = 5 + offset.North;
	wp.Position.East = 0 + offset.East;
	wp.ASL = 2;
	route.push_back(wp);
	// WP4 - raise to mission height + wait for deployment
	wp.Id = 4;
	wp.Position.North = DONT_CHANGE_VALUE;
	wp.Position.East = DONT_CHANGE_VALUE;
	wp.ASL = 4;
	wp.IsHoldState = true;
	wp.TimeFromPlan = DelayOnLaunch * (GroupSize) ;
	route.push_back(wp);
	// WP5 - return to traffic height
	wp.Id = 5;
	wp.Position.North = DONT_CHANGE_VALUE;
	wp.Position.East = DONT_CHANGE_VALUE;
	wp.ASL = 2;
	wp.IsHoldState = false;
	wp.TimeFromPlan = 0;
	route.push_back(wp);
	// WP6 - go to exit gateway
	wp.Id = 6;
	wp.Position.North = 2.5;
	wp.Position.East = 5;
	wp.ASL = 2;
	route.push_back(wp);
	// WP7 - go to arrival area
	wp.Id = 7;
	wp.Position.North = 0;
	wp.Position.East = 5;
	wp.ASL = 2;
	route.push_back(wp);
	// WP8 - go to landing area
	offset = GetLineOffset(PlatformId, GroupSize);
	wp.Id = 8;
	wp.Position.North = 0 + offset.North;
	wp.Position.East = 5 + offset.East;
	wp.ASL = 2;
	route.push_back(wp);
	// WP8 - go to landing area
	wp.Id = 9;
	wp.Position.North = DONT_CHANGE_VALUE;
	wp.Position.East = DONT_CHANGE_VALUE;
	wp.ASL = 0.2;
	route.push_back(wp);

	return route;


}

void FollowMe (FleetController* pFleet, int MasterId, int SlaveId)
{
	/*
	IPlatform_State		MasterState,
						SlaveState;
	Point2d				MasterPos,
						SlavePos;
	double				DeltaX,
						DeltaY,
						Dist;
	int					CONVOY_DISTANCE = 2;
	Waypoint			wp;

	pFleet->GetStatusFrom(MasterId, MasterState);
	pFleet->GetStatusFrom(SlaveId, SlaveState);
	MasterPos = MasterState.GetPosition();
	SlavePos = SlaveState.GetPosition();
	// calculate distance between 2 platforms:
	DeltaX = MasterPos.North - SlavePos.North;
	DeltaY = MasterPos.East - SlavePos.East;
	Dist = sqrt (DeltaX * DeltaX + DeltaY * DeltaY);

	wp.Id = 0;
	wp.IsHoldState = false;
	wp.TimeToWait = 0;
	if (Dist > CONVOY_DISTANCE)
	{
		printf ("slave too far %f\n", Dist);
		wp.Position = MasterPos;
		wp.ASL = MasterState.GetASL();
	}
	else
	{
		wp.Position = SlavePos;
		wp.ASL = SlaveState.GetASL();
	}

	pFleet->SendCmdTo(SlaveId, wp);
*/
}

// Diamond pattern
void BuildPattern (FleetController* pFleet)
{
	/*
	static double		MasterDirection,
						MasterSpeed;

	static Point2d		PrevPosition;
	static bool			IsFirstTime = true;
	IPlatform_State		MasterState;
	double				DeltaX,
						DeltaY;

	pFleet->GetStatusFrom(0, MasterState);

	if (IsFirstTime)
	{
		IsFirstTime = false;
		MasterDirection = 0;
		MasterSpeed = 0;

	}
	else
	{
		DeltaX = MasterState.Position.North - PrevPosition.North;
		DeltaY = MasterState.Position.East - PrevPosition.East;
		if (DeltaX != 0)
			MasterDirection = atan(DeltaY / DeltaX);
		else
			MasterDirection = 0;
	}

	PrevPosition = MasterState.Position;

*/




}

int main(int argc, char **argv)
{
	int				GroupSize,
					ReqPlatformId,
					DemoNumber;

	bool 			StopCmd,
					IsKeyActive;

	unsigned char 	key = ' ';

	FleetController*	pFleet;

	Waypoint		NewWp;

	DemoNumber = 0;
	IsKeyActive = true;
	StopCmd = false;
	GroupSize = -1;
	ReqPlatformId = 0;
	NewWp.ASL = 2;
	NewWp.Position.East = 0;
	NewWp.Position.North = 0;

	// Check if argument of ID is found
	if (argc > 1)
	{
		GroupSize = atoi( argv[1]);
		printf("FleetController for %d platforms Created\n", GroupSize);
	}
	else
	{
		printf("Invalid parameters: fleetcontroller_node <GroupSize>\n");
		return -1;
	}

	pFleet = new FleetController();
	pFleet->CreateFleet(GroupSize);

	//system ("roslaunch fleet spawn_platform.launch id:=0 x:=1 y:=4");

	// Set keyboard
	SetKeyboardInput();

	while (!StopCmd)
	{
		if (IsKeyActive)
		{
			read(STDIN_FILENO,&key,1);
			switch (key)
			{
				break;
			case 'q':
				printf ("Quit pressed\n");
				//pFleet->SendSimulatorCommand((int)GE_SIM_EXIT);
				StopCmd = true;
				break;
			case '1':		// Deploy uniform on mission area
			{
				DemoNumber = 1;
				Waypoint* routeArr;
				list<Waypoint> routeList;
				int routeLength;
				for (int i=0; i < GroupSize; i++)
				{
					printf ("New Route to platform %d\n", i);
					routeList = CreateDeployRoute(i, GroupSize);
					routeLength = routeList.size();
					routeArr = ConvertListToArray(routeList);

					pFleet->SendRouteTo(i, routeArr, routeLength);
				}
				break;
			}
			case '2':		// Convoy after leader
			{
//				PlatformMode mode;
//				mode.SetFlightMode (FLIGHT_MODE_MANUAL);
//				pFleet->SetMode(1, mode);
				DemoNumber = 2;
				break;
			}
			case 'r':
			case 'R':
				printf ("Reset Command\n");
				pFleet->SendSimulatorCommand((int)GE_SIM_RESET);

				break;
			case 's':
			case 'S':
				printf ("Stop Command\n");
				pFleet->SendSimulatorCommand((int)GE_SIM_STOP);
				break;
			case 'p':
			case 'P':
				printf ("Play Command\n");
				pFleet->SendSimulatorCommand((int)GE_SIM_PLAY);
				break;
			}
		}
		sleep(1);
		key = ' ';
		switch (DemoNumber)
		{
		case 2:
		{
			FollowMe(pFleet, 0, 1);
			FollowMe(pFleet, 1, 2);
			FollowMe(pFleet, 2, 3);

			break;
		}
		case 3:
			BuildPattern (pFleet);
			break;
		}
	}

	tcsetattr(STDOUT_FILENO,TCSANOW,&old_stdio);
	printf("FleetController_node closed\n");
	if (pFleet != 0)
		delete (pFleet);

	return 0;
}
