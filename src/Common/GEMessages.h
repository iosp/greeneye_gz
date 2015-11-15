#pragma once

#include "MissionIF.h"
#include "PlatformIF.h"

#define		GE_START_BYTES			0x5050
#define		GE_GS_ADDRESS				-1

#define 	MAX_ROUTE				15


typedef enum
{	
	GE_OP_CMD_SET_WP = 1,
	GE_OP_CMD_ROUTE,
	GE_OP_CMD_SET_SENSOR,
	GE_OP_REQ_STATUS,
	GE_OP_SET_MODE,
	GE_OP_SIM_CMD,	

	GE_OP_ACK	= 1001,
	GE_OP_STATUS
} GE_Msg_Opcodes;


typedef enum
{
	GE_SIM_EXIT = 1,
	GE_SIM_RESET,
	GE_SIM_STOP,
	GE_SIM_PLAY

} GE_Sim_Opcodes;




//////////////////////////////////////////////////////////////
//															//
//				GS Messages -> Platform						//
//															//
//////////////////////////////////////////////////////////////

typedef struct 
{
	int		StartBytes;
	int		MsgId;
	int		Src;
	int		Dst;
	int		MsgLength;
	int		Opcode;
} GEHeaderMsg;

typedef struct
{
	GEHeaderMsg	Header;
	Waypoint	WP;

} GECommandMsg;

typedef struct
{
	GEHeaderMsg	Header;
	int			NumWayPoints;
	Waypoint	WP [MAX_ROUTE];
} GERouteMsg;

typedef struct
{
	GEHeaderMsg Header;
} GEReqStatusMsg;

typedef struct
{
	GEHeaderMsg		Header;
	GE_Sim_Opcodes	SimCommand;
} GESimCmdMsg;

/*
typedef struct
{
	GEHeaderMsg			Header;
	PlatformMode		Mode;
} GEPlatformModeMsg;
*/
//////////////////////////////////////////////////////////////
//															//
//				Platform -> GS Messages						//
//															//
//////////////////////////////////////////////////////////////

typedef struct
{
	GEHeaderMsg		Header;
	IPlatform_State	State;

} GEStatusMsg;

typedef struct
{
	GEHeaderMsg		Header;
	bool			NAck;

} GEAckMsg;
