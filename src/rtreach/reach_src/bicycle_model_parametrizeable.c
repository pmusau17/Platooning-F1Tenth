#include <stdio.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include "bicycle_model_parametrizeable.h"

#include "main.h"
#include "face_lift_obstacle.h"
#include "util.h"
#include "simulate_bicycle.h"

// do face lifting with the given settings, iteratively improving the computation
// returns true if the reachable set of states is satisfactory according to the
// function you provide in LiftingSettings (reachedAtIntermediateTime, reachedAtFinalTime)
bool face_lifting_iterative_improvement_bicycle_dyn(int startMs, LiftingSettings* settings, REAL heading_input, REAL throttle,HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate,bool plot);


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



// Simulation 
REAL getSimulatedSafeTime(REAL start[4],REAL heading_input,REAL throttle)
{
	REAL stepSize = 0.02f;
	REAL rv = 0.0f;

	simulate_bicycle(start, heading_input,throttle,stepSize, shouldStop, (void*)&rv); // TODO: look here

	return rv;
}


void restartedComputation(int  *total_intermediate)
{
	// reset the counter of intermediate states 
	*total_intermediate = 0;
}


// called on states reached during the computation
bool intermediateState(HyperRectangle* r,HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate)
{
	if(*total_intermediate < max_intermediate)
	{
		VisStates[*total_intermediate] = *r;
	}
    *total_intermediate=*total_intermediate+1;	
	return true;
}


bool finalState(HyperRectangle* rect, HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate)
{
	if(*total_intermediate < max_intermediate)
	{
		VisStates[*total_intermediate] = *rect;
	}
    *total_intermediate=*total_intermediate+1;

	return true;
}


bool runReachability_bicycle_dyn(REAL* start, REAL simTime, REAL wallTimeMs, REAL startMs,REAL heading, REAL throttle,HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate,bool plot)
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

	
	

	set.initialStepSize = iss; 
	set.maxRectWidthBeforeError = 100;

	set.reachedAtFinalTime = finalState;
	set.reachedAtIntermediateTime = intermediateState;
	set.restartedComputation = restartedComputation; 

    return face_lifting_iterative_improvement_bicycle_dyn(startMs, &set,heading, throttle,VisStates, total_intermediate,max_intermediate,plot);
}

bool check_safety(HyperRectangle* rect, REAL (*cone)[2])
{
	
	REAL l1[2] = {rect->dims[0].min,rect->dims[1].max};
    REAL r1[2] = {rect->dims[0].max,rect->dims[1].min};

	REAL l2[2] = {cone[0][0],cone[1][1]};
    REAL r2[2] = {cone[0][1],cone[1][0]};

	if (l1[0] >= r2[0] || l2[0] >= r1[0])
	{
        return true; 
	}
    if (l1[1] <= r2[1] || l2[1] <= r1[1])
	{
        return true;
	}

	return false;
}