// Patrick Musau
// 11-2020
// UUV model header

#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include "main.h"
#include "geometry.h"
#include <stdbool.h>



// run reachability for a given wall timer (or iterations if negative)
bool runReachability_obstacle(REAL* start, REAL simTime, REAL wallTimeMs, REAL startMs,REAL v_x, REAL v_y);
HyperRectangle runReachability_obstacle_vis(REAL* start, REAL simTime, REAL wallTimeMs, REAL startMs,REAL v_x, REAL v_y);
REAL getSimulatedSafeTime(REAL start[2],REAL v_x, REAL v_y);

#endif