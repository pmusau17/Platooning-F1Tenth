// Patrick Musau
// 08-2020


#include <stdio.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>

#include "dynamics_bicycle.h"
#include "main.h"
#include "face_lift.h"
#include "util.h"



// do face lifting with the given settings, iteratively improving the computation
// returns true if the reachable set of states is satisfactory according to the
// function you provide in LiftingSettings (reachedAtIntermediateTime, reachedAtFinalTime)
bool face_lifting_iterative_improvement_bicycle(int startMs, LiftingSettings* settings, REAL heading_input, REAL throttle);
REAL get_derivative_bounds_bicycle(HyperRectangle* rect, int faceIndex,REAL heading_input, REAL throttle);


// Constants necessary to guarantee loop termination.
// These bound the values of the derivatives
const REAL MAX_DER_B = 99999;
const REAL MIN_DER_B = -99999;

// for benchmarking purposes having the iterations at quit is huge
int iterations_at_quit = 0;

// make a face's neighborhood of a given width
// At each dimension, there are two faces corresponding to that dimension, minimum_face and maximum_face
// For example, Rect: 0 <= x <= 2: the minimum_face is at x = 0 (a point in this case), the maximum_face is at x = 2
// For two dimensional Rectangle:     0 <= x1 <= 2; 1 <= x2 <= 3: at the dimension 1 (i.e., x1 axis) the minimum face
// is a line x1 = 0, 1 <= x2 <= 3 and the maximum face is a line x1 = 2, 1 <= x2 <= 3

void make_neighborhood_rect_bicycle(HyperRectangle* out, int f, HyperRectangle* bloatedRect, HyperRectangle* originalRect, REAL nebWidth)
{
	*out = *bloatedRect;

	bool isMin = (f % 2) == 0;
	int d = f / 2;

	// flatten
	// The derivatives are evaluated along the face
	// so what the next line does is take face value based on the dimension 
	// and whether the face is oriented to the negative or positive direction, respectively
	// e_i+ = x_i = ui, e_i- = l_i

	// select the negative face for this dimension
	if (isMin)
	{
		out->dims[d].min = originalRect->dims[d].min;
		out->dims[d].max = originalRect->dims[d].min;
	}
	else
	{
		// select the positive face for this dimension
		out->dims[d].min = originalRect->dims[d].max;
		out->dims[d].max = originalRect->dims[d].max;
	}

	// depending on the value returned by the derivative 
	// extend the dimensions by the derivative
	// if its a negative facing face, the negative directions move it outward 
	// and vice versa
	// swap if nebWidth was negative

	if (nebWidth < 0)
		out->dims[d].min += nebWidth;
	else
		out->dims[d].max += nebWidth;
}



// do a single face lifting operation
// et (error tracker) is set if you want to track the sources of errors, can be null
// returns time elapsed

REAL lift_single_rect_bicycle(HyperRectangle* rect, REAL stepSize, REAL timeRemaining, REAL heading_input, REAL throttle)
{
  
	////////////////////////////////////////////////
	// estimate the widths of the neighborhoods   //
	// construct bloated rect (for neighborhoods) //

	// The reason we have this bloated rect, which is a copy of the rectangle for which we 
	// are doing the face_lifting operations, is if we need to recompute the derivatives
	// for the facelifting

	HyperRectangle bloatedRect = *rect;
	REAL nebWidth[NUM_FACES];

	// initially, estimate nebWidth based on the derivative in the center of the rectangle we care about

	for (int f = 0; f < NUM_FACES; ++f)
		nebWidth[f] = 0;

	bool needRecompute = true;
	REAL minNebCrossTime;
	REAL ders[NUM_FACES]; // array that stores each derivative for each face
	
	
	while (needRecompute)
	{
		needRecompute = false;
		minNebCrossTime = DBL_MAX;

		for (int f = 0; f < NUM_FACES; ++f)
		{
			int dim = f / 2;
			bool isMin = (f % 2) == 0;

			HyperRectangle faceNebRect;

			// make candidate neighborhood
			make_neighborhood_rect_bicycle(&faceNebRect, f, &bloatedRect, rect, nebWidth[f]);
			
			// test derivative inside neighborhood

			// the projection of the derivative on the outward norm e_i- is -fi(x) and fi(x) for e_i
			REAL der = get_derivative_bounds_bicycle(&faceNebRect,f, heading_input, throttle);

	
			// so we cap the derivative at 999999 and min at the negative of that.
			if (der > MAX_DER_B) {
				der = MAX_DER_B;
                        }
			else if (der < MIN_DER_B) {
				der = MIN_DER_B;
                        }

			REAL prevNebWidth = nebWidth[f];
			
			REAL newNebWidth = der * stepSize; // the projection of the derivative is tempered by the stepSize

			// if it is a negative facing face the derivative is negative if it is less than 0.
			// if it is a positive facing face the derivative has to be positive to grow and outward.

			bool grewOutward = (isMin && newNebWidth < 0) || (!isMin && newNebWidth > 0); 

			// check if the previous nebWidth grewOutward.

			bool prevGrewOutward = (isMin && prevNebWidth < 0) || (!isMin && prevNebWidth > 0);

			// prevent flipping from outward face to inward face
			if (!grewOutward && prevGrewOutward)
			{
				newNebWidth = 0;
				der = 0;
			}

			// if flipping from inward to outward
			if (!prevGrewOutward && grewOutward) {
				needRecompute = true;
                        }

			// 2nd condition to recompute, der doubled (which means neb width is twice what it was before)
			if (fabs(newNebWidth) > 2 * fabs(prevNebWidth)) {
				needRecompute = true;
                        }

			// adjust bloated rect only if we are requiring a later recomputation
			if (needRecompute)
			{
				nebWidth[f] = newNebWidth;

				if (isMin && nebWidth[f] < 0)
					bloatedRect.dims[dim].min = rect->dims[dim].min + nebWidth[f];
				else if (!isMin && nebWidth[f] > 0)
					bloatedRect.dims[dim].max = rect->dims[dim].max + nebWidth[f];

				
			}
			else
			{
				// might be the last iteration, compute min time to cross face

				// clamp derivative if it changed direction
				// this means along the face it's inward, but in the neighborhood it's outward
				if (der < 0 && prevNebWidth > 0) {
					der = 0;
                                }
				else if (der > 0 && prevNebWidth < 0) {
					der = 0;
                                }

				if (der != 0)
				{
					REAL crossTime = prevNebWidth / der;

					if (crossTime < minNebCrossTime) {
						minNebCrossTime = crossTime;
                                        }
				}

				ders[f] = der;
			}

		}
	}


	// just as a note the minTime to cross is the prevNebwidth / der
	// the nebWidth btw is stepSize * der
	if (minNebCrossTime * 2 < stepSize)
	{
		error_exit("minNebCrossTime is less than half of step size.");
	}

	////////////////////////////////////////
	// lift each face by the minimum time //

	REAL timeToElapse = minNebCrossTime;


	// subtract a tiny amount time due to multiplication / division rounding
	timeToElapse = timeToElapse * 99999 / 100000;

	if (timeRemaining < timeToElapse) {
		timeToElapse = timeRemaining;
        }

	// do the lifting
	for (int d = 0; d < NUM_DIMS; ++d)
	{
		rect->dims[d].min += ders[2*d] * timeToElapse;
		rect->dims[d].max += ders[2*d+1] * timeToElapse;
	}

	if (!hyperrectangle_contains(&bloatedRect, rect, true))
	{
		//printf("error occurred when debugNumCalls = %d\n", debugNumCalls);
		//printf("rect = ");
		//println(&debug_initialRect);

		error_exit("lifted rect is outside of bloated rect");
	}

	return timeToElapse;
}


bool face_lifting_iterative_improvement_bicycle(int startMs, LiftingSettings* settings,REAL heading_input, REAL throttle)
{
	bool rv = false;
	bool lastIterationSafe = false;

	// set the start time
	struct timeval start;
	int elapsedTotal;
	gettimeofday(&start, NULL);
	set_error_print_params(settings);

	// Get the settings from the facelifting settings
	REAL stepSize = settings->initialStepSize;

	int iter = 0; // number of iterations
	int previous_iter =1;
	int elapsed_prev = 0;
	int next_iter_estimate = 0;

	while (true)
	{
		iter++;
		bool safe = true; // until proven otherwise

		// if we've cut the step size way too far where floating point error may be a factor
		if (stepSize < 0.0000001)
		{
			DEBUG_PRINT("Quitting from step size too small: stepSize: %0.10f at iteration: %d\n\r", stepSize, iter);
			rv = false;
			break;
		}

		// this is primarily used in the plotting functions, so that we only plot the last iteration
		if (settings->restartedComputation)
		{
			settings->restartedComputation();
        }

		// This function gets the reachtime passed from the settings 
		REAL timeRemaining = settings->reachTime;

		// Get the initial set from which to perform reachability analysis.
		HyperRectangle trackedRect = settings->init;

		// Create a new hyperrectangle
		HyperRectangle hull;

		// I want to visualize an over-approximation of the over-all reachset too
		HyperRectangle total_hull = trackedRect;

		// compute reachability up to split time
		while (safe && timeRemaining > 0)
		{
			// reachedAtIntermediateTime is a function that checks the current hyper-rectangle against the safety specification,
			// whatever that might be
			if (settings->reachedAtIntermediateTime) 
			{
				hull = trackedRect;
            }
			
			// debug changed so error tracker is always passed in (see note)
			REAL timeElapsed = lift_single_rect_bicycle(&trackedRect, stepSize, timeRemaining, heading_input, throttle);

			// if we're not even close to the desired step size
			if (hyperrectange_max_width(&trackedRect) > settings->maxRectWidthBeforeError)
			{
				DEBUG_PRINT("maxRectWidthBeforeError exceeded at time %f, rect = ",
						settings->reachTime - timeRemaining);
				#if DEBUG
				println(&trackedRect);
				#endif
				// step size is too large, make it smaller and recompute
				safe = false;
			}
			else if (settings->reachedAtIntermediateTime)
			{
				hyperrectangle_grow_to_convex_hull(&hull, &trackedRect);
				hyperrectangle_grow_to_convex_hull(&total_hull, &trackedRect);

				safe = safe && settings->reachedAtIntermediateTime(&hull);
			}

			if (timeElapsed == timeRemaining && settings->reachedAtFinalTime)
				safe = safe && settings->reachedAtFinalTime(&trackedRect);

			timeRemaining -= timeElapsed;

		} // This is the end of this first while loop
		  // This does the reachset computation for the reachtime
		  // it continues until the simulation time is over, or we encounter an unsafe state,
		  // whichever occurs first. 


		// Don't do another iteration unless you want to miss the deadline
		int now = milliseconds2(&start);
		elapsedTotal = now;
		previous_iter = elapsedTotal - elapsed_prev;
		// its O(2^N) in terms of box checking so have to scale the next iteration by 2 and add 1ms (for over-approximating how long it takes to compute the reachset)
		if(previous_iter==0)
			next_iter_estimate = 2;
		else
		{
			if((previous_iter * 2+1)<next_iter_estimate)
				next_iter_estimate = next_iter_estimate * 2;
			else
				next_iter_estimate  = previous_iter * 2+1;
		}
		
		elapsed_prev = elapsedTotal;
		//DEBUG_PRINT("elaspedTotal :%d, previous_iter: %d, projected_next_iter: %d\n\r", elapsedTotal,previous_iter,next_iter_estimate);
		if (settings->maxRuntimeMilliseconds > 0)
		{
			int remaining = settings->maxRuntimeMilliseconds - elapsedTotal;

			if(remaining<0)
				DEBUG_PRINT("remaining: %d\r\n",remaining);
			if (remaining <= next_iter_estimate)
			{
				// we've exceeded our time, use the result from the last iteration
				// note in a real system you would have an interrupt or something to cut off computation
				//DEBUG_PRINT("Quitting from runtime maxed out\n\r");
				rv = lastIterationSafe;
				//println(&trackedRect);
				break;
			}
		} 
		else
		{
			// runtime was negative, split a fixed number of times
			if (settings->maxRuntimeMilliseconds++ == 0)
			{
				DEBUG_PRINT("Splitting\n\r");
				rv = safe;
				break;
			}
		}

		lastIterationSafe = safe;

		// apply error-reducing strategy
		stepSize /= 2;
	}

	iterations_at_quit = iter;
	// DEBUG_PRINT("%dms: stepSize = %f\n",	elapsedTotal, stepSize);
	// DEBUG_PRINT("iterations at quit: %d\n\r", iter);

	return rv;
}

