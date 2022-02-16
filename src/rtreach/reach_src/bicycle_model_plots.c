#include "bicycle_model.h"
#include "main.h"
#include "face_lift.h"
#include "util.h"
#include "simulate_bicycle_plots.h"
#include <stdio.h>

static FILE* f_initial;
static FILE* f_intermediate;
static FILE* f_final;


// global variable definitions 
double maxTime = 2.0;

// do face lifting with the given settings, iteratively improving the computation
// returns true if the reachable set of states is satisfactory according to the
// function you provide in LiftingSettings (reachedAtIntermediateTime, reachedAtFinalTime)
// returns the convex hull of the reachset

HyperRectangle face_lifting_iterative_improvement_bicycle_vis(int startMs, LiftingSettings* settings, REAL heading_input, REAL throttle,bool plot);

// function that stops simulation after two seconds
bool shouldStop(REAL state[NUM_DIMS], REAL simTime, void* p)
{
	bool rv = false;
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
	REAL stepSize = 0.002f;
	REAL rv = 0.0f;

	simulate_bicycle_plots(start, heading_input,throttle,stepSize, shouldStop, (void*)&rv); // TODO: look here

	//DEBUG_PRINT("time until simulation reaches safe state = %f\n", rv);

	return rv;
}

void close_files(bool closeInitial)
{
	if (closeInitial && f_initial)
		fclose(f_initial);

	if (f_intermediate)
		fclose(f_intermediate);

	if (f_final)
		fclose(f_final);
}


void open_files(bool openInitial)
{
	if (openInitial)
		f_initial = fopen("bicycle_initial.gnuplot.txt", "w");

	f_intermediate = fopen("bicycle_intermediate.gnuplot.txt", "w");
	f_final = fopen("bicycle_final.gnuplot.txt", "w");

	if (!f_initial || !f_intermediate || !f_final)
	{
		close_files(true);

		error_exit("error opening files");
	}
}

// if computation restarts we close and reopen files
void restartedComputation()
{
	// close and open the files to clear them
	close_files(false);
	open_files(false);

	// reset the counter of intermediate states 
	num_intermediate = 0;
	total_intermediate = 0;
	final_hull = false;
}

// styleIndex: 0 = init, 1 = intermediate, 2 = final
void hyperrectangle_to_file(FILE* fout, HyperRectangle* r, int styleIndex)
{
	if (fout)
	{
        // select which dimesnions you want to plot
		int X_DIM = 0;
		int Y_DIM = 1;

		const char* styleStr[] =
		{
			"set label ' Init' at %f, %f point pointtype 3 lc rgb 'blue' tc rgb 'blue'",
			"set obj rect from %f, %f to %f, %f fc rgbcolor 'dark-green' fs solid 0.2 \n",
			"set obj rect from %f, %f to %f, %f fc rgbcolor 'red' fs solid 0.3\n",
		};

		fprintf(fout, styleStr[styleIndex], r->dims[X_DIM].min, r->dims[Y_DIM].min,
				r->dims[X_DIM].max, r->dims[Y_DIM].max);
	}
}


// called on states reached during the computation
// this function basically says that all intermediate sates are safe
// during the reachset computation
bool intermediateState(HyperRectangle* r)
{

	hyperrectangle_to_file(f_intermediate, r,1);
	
	// add state to array for plotting
	if(num_intermediate < MAX_INTERMEDIATE)
	{
		VisStates[num_intermediate] = *r;
		num_intermediate++;
	}	
	total_intermediate++;

	return true;
}

// This function enumerates all of the corners of the current HyperRectangle and 
// returns whether or not any of the points lies outside of the ellipsoid. The corner
// Thing is only really helpful for linear models.
bool finalState(HyperRectangle* rect)
{
	hyperrectangle_to_file(f_final, rect,2);
	// final state to array if space permits add state to array for plotting
	if(num_intermediate < MAX_INTERMEDIATE && !final_hull)
	{
		VisStates[num_intermediate] = *rect;
		num_intermediate++;
	}
	final_hull = true;
	total_intermediate++;

	return false;
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

    open_files(true);
	hyperrectangle_to_file(f_initial, &set.init, 0);

	return face_lifting_iterative_improvement_bicycle_vis(startMs, &set,heading_input, throttle,true);
}

