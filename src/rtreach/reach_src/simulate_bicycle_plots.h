// Patrick Musau
// 4-2014
// Simulate Header for plotting

#ifndef SIMULATE_H_
#define SIMULATE_H_

#include "geometry.h"
#include "main.h"
#include <stdbool.h>

// maximum number of intermediate hyper-rectangles to display 
#define MAX_INTERMEDIATE 2000


extern double maxTime;
extern struct HyperRectangle VisStates[MAX_INTERMEDIATE];
extern int num_intermediate;
extern int total_intermediate;


// prevents from including the hull into VisStates
// largely because I call finalState twice one to plot the final state
// and then a second time to plot the hull
extern bool final_hull;


// simulate dynamics using Euler's method
void simulate_bicycle_plots(REAL point[NUM_DIMS], REAL heading_input, REAL throttle,
			  REAL stepSize,
			  bool (*shouldStop)(REAL state[NUM_DIMS], REAL simTime, void* p),
			  void* param);

#endif