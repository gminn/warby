#include "warby.h"

typedef enum
{
    Init_State,
	Transit_State,
	Obstacle_Detect_Transit_State,
} eSystemState;

typedef enum
{
	Light_Detect_Event,
	Light_Out_Of_Range_Event,
	Obstacle_Detect_Event,
} eSystemEvent;

