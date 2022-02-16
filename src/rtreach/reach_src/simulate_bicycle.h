// Patrick Musau
// 4-2014
// Simulate Header

#ifndef SIMULATE_H_
#define SIMULATE_H_

#include "dynamics_bicycle_model.h"
#include "geometry.h"
#include "main.h"
#include <stdbool.h>

// simulate dynamics using Euler's method
void simulate_bicycle(REAL point[NUM_DIMS], REAL heading_input, REAL throttle,
			  REAL stepSize,
			  bool (*shouldStop)(REAL state[NUM_DIMS], REAL simTime, void* p),
			  void* param);

#endif
