#include "bicycle_model.h"
#include "main.h"
#include "face_lift.h"
#include "util.h"
#include "bicycle_safety.h"
#include "simulate_bicycle_plots.h"
#include <stdio.h>
#include <string.h> 
#include <stdlib.h>


// a note from the f1tenth simulator 
// the car is 0.5 m long in the x direction 
// 0.3 long in the y direction

// do face lifting with the given settings, iteratively improving the computation
// returns true if the reachable set of states is satisfactory according to the
// function you provide in LiftingSettings (reachedAtIntermediateTime, reachedAtFinalTime)

// visualization version, returns convex hull
HyperRectangle face_lifting_iterative_improvement_bicycle_vis(int startMs, LiftingSettings* settings, REAL heading_input, REAL throttle,bool plot);


// function that stops simulation after two seconds
bool shouldStop(REAL state[NUM_DIMS], REAL simTime, void* p)
{
	bool rv = false;
    REAL maxTime = 2.0f;
    // stop if the maximum simulation time 
	if (simTime >= maxTime)
	{
		rv = true;

		REAL* stopTime = (REAL*)p;
		*stopTime = -1;
	}

	return rv;
}


// if computation restarts we close and reopen files
void restartedComputation()
{
	// reset the counter of intermediate states 
	num_intermediate = 0;
	total_intermediate = 0;
	final_hull = false;
}

// called on states reached during the computation
bool intermediateState(HyperRectangle* r)
{
	bool allowed = true;
	//const REAL FIFTEEN_DEGREES_IN_RADIANS = 0.2618;

	// bloat the box for the width of the car
	// r->dims[0].min = r->dims[0].min  - 0.25;
	// r->dims[0].max = r->dims[0].max  + 0.25;
	// r->dims[1].min = r->dims[1].min  - 0.15;
	// r->dims[1].max = r->dims[1].max  + 0.15;

	
	

	// allowed = check_safety_obstacles(r);
	
	// if(allowed)
	// {
	// 	allowed = check_safety_wall(r);
	// }
	
	// // reset it
	// r->dims[0].min = r->dims[0].min  + 0.25;
	// r->dims[0].max = r->dims[0].max  - 0.25;
	// r->dims[1].min = r->dims[1].min  + 0.15;
	// r->dims[1].max = r->dims[1].max  - 0.15;

	// copy intermediate state into array
	// add state to array for plotting
	if(num_intermediate < MAX_INTERMEDIATE)
	{
		VisStates[num_intermediate] = *r;
		num_intermediate++;
	}	
	total_intermediate++;

	//if(!allowed)
		// printf("unsafe....\n");
	return allowed;
}

// This function enumerates all of the corners of the current HyperRectangle and 
// returns whether or not any of the points lies outside of the ellipsoid
bool finalState(HyperRectangle* rect)
{

	return intermediateState(rect);
}


// reachability analysis
HyperRectangle runReachability_bicycle_vis(REAL* start, REAL simTime, REAL wallTimeMs, REAL startMs,REAL heading_input, REAL throttle)
{
	LiftingSettings set;
	for (int d = 0; d < NUM_DIMS; ++d)
	{
		set.init.dims[d].min = start[d];
		set.init.dims[d].max = start[d];
	}

	set.reachTime = simTime;
	set.maxRuntimeMilliseconds = wallTimeMs;

	REAL iss = set.reachTime;
	iss = iss * 0.10f;

	set.initialStepSize = iss; //set.reachTime / 10.0f;
	set.maxRectWidthBeforeError = 100;

	set.reachedAtFinalTime = finalState;
	set.reachedAtIntermediateTime = intermediateState;
    set.restartedComputation = restartedComputation; 

	return face_lifting_iterative_improvement_bicycle_vis(startMs, &set,heading_input, throttle,false);
}
