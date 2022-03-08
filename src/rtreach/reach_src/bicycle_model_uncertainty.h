// Patrick Musau
// Vehicle Bicycle Uncertainty Model Header

#ifndef BICYCLE_UNCERTAIN_H_
#define BICYCLE_UNCERTAIN_H_

#include "main.h"
#include "geometry.h"
#include <stdbool.h>



// run reachability for a given wall timer (or iterations if negative)
bool runReachability_bicycle_uncertain(REAL start[][2], REAL simTime, REAL wallTimeMs, REAL startMs,REAL heading, REAL throttle, REAL parameter_uncertainty, REAL disturbance[][2], HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate,bool plot);
bool check_safety(HyperRectangle* rect, REAL (*cone)[2]);

#endif