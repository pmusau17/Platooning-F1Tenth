#include <stdio.h>
#include "simulate_bicycle.h"
#include "dynamics_bicycle_model.h"


// simulate dynamics using Euler's method
void simulate_bicycle(REAL startPoint[NUM_DIMS], REAL heading_input, REAL throttle,
			  REAL stepSize,
			  bool (*shouldStop)(REAL state[NUM_DIMS], REAL simTime, void* p),
			  void* param)
{	
	// define the point 
	REAL point[NUM_DIMS];

	// initialize the point array with the values of start point
	for (int d = 0; d < NUM_DIMS; ++d) {
		point[d] = startPoint[d];
        }

	// declare a hyperRectangle: an array of intervals
	HyperRectangle rect;
//	REAL time = stepSize; // was 0.0f

	REAL time = 0.0f;

	while (true)
	{	
		
        // my assumption here is that if the point is within the ellipsoid then we don't have to do any simulation
		if (shouldStop(point, time, param)) {
			DEBUG_PRINT("Quitting simulation: time: %f, stepSize: %f\n\r", time, stepSize);
			break;
		}

		// initialize the hyper-rectangle. Since we are doing simulation of a point then 
		// interval.min and interval.max are the same point
		for (int d = 0; d < NUM_DIMS; ++d) {
			rect.dims[d].min = rect.dims[d].max = point[d];
		}

		// euler's method
		for (int d = 0; d < NUM_DIMS; ++d)
		{
			REAL der = get_derivative_bounds_bicycle(&rect, 2*d,heading_input,throttle);

			point[d] += stepSize * der;
		}

		time += stepSize;
	}

	printf("If you keep the same input for the next %f s, the state will be: \n [%f,%f,%f,%f] \n", time-stepSize,point[0],point[1],point[2],point[3]);
}
