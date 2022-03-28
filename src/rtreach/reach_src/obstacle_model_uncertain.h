// Patrick Musau
// 03-2022
// Dynamic Obstacle Model Uncertainty Model 

#ifndef OBSTACLE_UNCERTAIN_H_
#define OBSTACLE_UNCERTAIN_H_

#include "main.h"
#include "geometry.h"
#include <stdbool.h>


HyperRectangle runReachability_obstacle_uncertain(REAL start[][2], REAL simTime, REAL wallTimeMs, REAL startMs,REAL v_x[2], REAL v_y[2], HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate,bool plot);

#endif