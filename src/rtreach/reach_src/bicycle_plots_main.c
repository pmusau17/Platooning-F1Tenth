#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "bicycle_model.h"
#include "simulate_bicycle_plots.h"
#include "util.h"

int main(int argc, char** argv)
{
	int rv = 0;

	if (argc != 9)
	{
		printf("Usage: rt_reach (milliseconds-runtime) (seconds-reachtime) (x) (y) (linear velocity) (heading) (throttle control input) (heading control input)\n");
		printf("If milliseconds is negative, it will split a fixed number of times\n");
		rv = 1;
	}
	else
	{
		int ms = atoi(argv[1]);
        
		if (ms < 0)
			printf("splitting time step exactly %i times ", -ms);
		else
		{
			printf("Using time bound. Please note that the times will be inflated and noisy, "
				"because we're outputting the reach set to files on disk. Normally you wouldn't have this overhead.\n\n");
			printf("Running for %i ms ", ms);
		}
        
        double reachTime = atof(argv[2]);
		maxTime = reachTime;
        printf("Reach time is %f seconds, simTime is %f seconds\n", reachTime,maxTime);

		// setttings for pendulum
		double startState[4] = {atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6])};
		REAL control_input[2] = {0.0,0.0};

		control_input[0] = atof(argv[7]);
        control_input[1] = atof(argv[8]);

		DEBUG_PRINT("runtime: %d ms\n\rx_0[0]: %f\n\rx_0[1]: %f\n\rx_0[2]: %f\n\rx_0[3]: %f\n\ru_0[0]: %f\n\ru_0[1]: %f\n\r\n", ms, startState[0], startState[1], startState[2], startState[3],control_input[0],control_input[1]);

		REAL delta = control_input[1];
    	REAL u = control_input[0];
    	// simulate the car with a constant input passed from the command line
    	getSimulatedSafeTime(startState,delta,u);
    	printf("\n");
		
    	// run reachability analysis test 
    	HyperRectangle hull = runReachability_bicycle_vis(startState, reachTime, ms, milliseconds(),delta,u);
		println(&hull);
	    DEBUG_PRINT("Number of Iterations: %d\n",iterations_at_quit);
	}

	return rv;
}
