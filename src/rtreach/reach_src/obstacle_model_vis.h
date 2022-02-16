// Patrick Musau
// 11-2020
// UUV model header

#ifndef OBSTACLE_VIS_H_
#define OBSTACLE_VIS_H_

#include "main.h"
#include "geometry.h"
#include <stdbool.h>



// run reachability for a given wall timer (or iterations if negative)
HyperRectangle runReachability_obstacle_vis(REAL* start, REAL simTime, REAL wallTimeMs, REAL startMs,REAL v_x, REAL v_y, HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate,bool plot);
HyperRectangle runReachability_obstacle_vis_uncertain(REAL start[][2], REAL simTime, REAL wallTimeMs, REAL startMs,REAL v_x, REAL v_y, HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate,bool plot);
REAL getSimulatedSafeTime(REAL start[2],REAL v_x, REAL v_y);

#endif