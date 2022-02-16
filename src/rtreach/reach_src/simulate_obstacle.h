// Patrick Musau
// 11-2020
// Simulate Header

#ifndef SIMULATE_OBSTACLE_H_
#define SIMULATE_OBSTACLE_H_

#include "geometry.h"
#include "main.h"
#include <stdbool.h>

// simulate dynamics using Euler's method
void simulate_obstacle(REAL point[NUM_DIMS], REAL v_x, REAL v_y,
			  REAL stepSize,
			  bool (*shouldStop)(REAL state[NUM_DIMS], REAL simTime, void* p),
			  void* param);

#endif
