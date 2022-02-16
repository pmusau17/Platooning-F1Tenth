#include <stdio.h>
#include "simulate_bicycle_plots.h"
#include "dynamics_bicycle_model.h"
#include "util.h"




static FILE* simulation_file;

// declarations
void open_sim_file(bool openInitial);
void hyperrectangle_to_point_file(FILE* fout, REAL *point);
bool close_file(bool closeInitial);


// functions that need only exist in this file
void open_sim_file(bool openInitial)
{
	if (openInitial)
		simulation_file = fopen("bicycle_simulation.dat", "w");

	if (!simulation_file)
	{
		close_file(true);

		error_exit("error opening files");
	}
}

bool close_file(bool closeInitial)
{
	if (closeInitial && simulation_file)
		return fclose(simulation_file)==0;
	else
		return false;
}



void hyperrectangle_to_point_file(FILE* fout, REAL *point)
{
	if (fout)
	{
        // select which dimesnions you want to plot
		int X_DIM = 0;
		int Y_DIM = 1;

		const char* styleStr = "%f\t%f\n";
		//printf("%f\t%f\n",*point[X_DIM], *point[Y_DIM]);

		fprintf(fout, styleStr, point[X_DIM], point[Y_DIM]);
	}
}


// simulate dynamics using Euler's method
void simulate_bicycle_plots(REAL startPoint[NUM_DIMS], REAL heading_input, REAL throttle,
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


	// record start state of simulation
	hyperrectangle_to_point_file(simulation_file, point);

	// declare a hyperRectangle: an array of intervals
	HyperRectangle rect;
//	REAL time = stepSize; // was 0.0f


    // open the file
    open_sim_file(true);
	

	REAL time = 0.0f;

	while (true)
	{	
		
        // my assumption here is that if the point is within the ellipsoid then we don't have to do any simulation
		if (shouldStop(point, time, param)) {
			DEBUG_PRINT("Quitting simulation: time: %f, stepSize: %f\n\r", time, stepSize);
			close_file(true);
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

		// log the point to a file
		hyperrectangle_to_point_file(simulation_file, point);

		time += stepSize;
	}

    
	printf("If you keep the same input for the next %f s, the state will be: \n [%f,%f,%f,%f] \n", time-stepSize,point[0],point[1],point[2],point[3]);
}
