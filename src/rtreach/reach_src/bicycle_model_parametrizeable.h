// Patrick Musau
// Vehicle Bicycle model header

#ifndef BICYCLE_PARAM_H_
#define BICYCLE_PARAM_H_

#include "main.h"
#include "geometry.h"
#include <stdbool.h>



// run reachability for a given wall timer (or iterations if negative)
bool runReachability_bicycle_dyn(REAL* start, REAL simTime, REAL wallTimeMs, REAL startMs,REAL heading, REAL throttle,HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate,bool plot);
REAL getSimulatedSafeTime(REAL start[4],REAL heading_input, REAL throttle);
bool check_safety(HyperRectangle* rect, REAL (*cone)[2]);

#endif