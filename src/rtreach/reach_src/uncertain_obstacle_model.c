#include "obstacle_model_uncertain.h"
#include "main.h"
#include "face_lift_obstacle.h"
#include "util.h"
#include <stdio.h>
#include <string.h> 
#include <stdlib.h>

// do face lifting with the given settings, iteratively improving the computation
// returns true if the reachable set of states is satisfactory according to the
// function you provide in LiftingSettings (reachedAtIntermediateTime, reachedAtFinalTime)
// returns the convex hull of the reachset

HyperRectangle face_lifting_iterative_improvement_uncertain(int startMs, LiftingSettings* settings, REAL v_x[2], REAL v_y[2], HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate,bool plot);

// if computation restarts we close and reopen files
void restartedComputation(int  *total_intermediate)
{
	
	// reset the counter of intermediate states 
	*total_intermediate = 0;
}

// called on states reached during the computation
// this function basically says that all intermediate sates are safe
// during the reachset computation
bool intermediateState(HyperRectangle* r,HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate)
{

	// hyperrectangle_to_file(f_intermediate, r,1);
	
	// add state to array for plotting
	if(*total_intermediate < max_intermediate)
	{
		VisStates[*total_intermediate] = *r;
	}
    *total_intermediate=*total_intermediate+1;	
	return true;
}

// This function enumerates all of the corners of the current HyperRectangle and 
// returns whether or not any of the points lies outside of the ellipsoid. The corner
// Thing is only really helpful for linear models.
bool finalState(HyperRectangle* rect, HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate)
{
	// hyperrectangle_to_file(f_final, rect,2);
	// final state to array if space permits add state to array for plotting
	if(*total_intermediate < max_intermediate)
	{
		VisStates[*total_intermediate] = *rect;
	}
    *total_intermediate=*total_intermediate+1;

	return true;
}


HyperRectangle runReachability_obstacle_uncertain(REAL start[][2], REAL simTime, REAL wallTimeMs, REAL startMs,REAL v_x[2], REAL v_y[2], HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate,bool plot)
{
	LiftingSettings set;
	for (int d = 0; d < NUM_DIMS; ++d)
	{
		set.init.dims[d].min = start[d][0];
		set.init.dims[d].max = start[d][1];
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

	return face_lifting_iterative_improvement_uncertain(startMs, &set,v_x, v_y,VisStates, total_intermediate,max_intermediate,plot);

}