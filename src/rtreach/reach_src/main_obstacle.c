// example call: ./obstacle 10 0 0 1.0 0.0
// example call output:
// started!
// Argc: 7
// runtime: 10 ms
// x_0[0]: 0.0
// x_0[1]: 0.0
// u_0[0]: 1.0
// u_0[1]: 0.0

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "util.h"
#include "main.h"
#include "obstacle_model.h"
#include "simulate_obstacle.h"

const int state_n = 2; // state dimension

int main( int argc, const char* argv[] )
{
	DEBUG_PRINT("started!\n\r");

	int runtimeMs = 0;
	REAL startState[2] = {0.0, 0.0};
    REAL v_u[2] = {0.0,0.0};

	DEBUG_PRINT("Argc: %d\n\r", argc);

	if (argc < 6) {
		printf("Error: not enough input arguments!\n\r");
		return 0;
	}
	else {
		runtimeMs = atoi(argv[1]);
		startState[0] = atof(argv[2]);
		startState[1] = atof(argv[3]);
        v_u[0] = atof(argv[4]);
        v_u[1] = atof(argv[5]);
		DEBUG_PRINT("runtime: %d ms\n\rx_0[0]: %f\n\rx_0[1]: %f\n\ru_0[0]: %f\n\ru_0[1]: %f\n\r\n", runtimeMs, startState[0], startState[1],v_u[0],v_u[1]);
	}

    
	REAL v_x = v_u[0];
    REAL v_y = v_u[1];
   
    // simTime 
    REAL timeToSafe = 2.0;
    // startMs 
    int startMs = 0;



    getSimulatedSafeTime(startState,v_x,v_y);
    printf("\n");

    // run reachability analysis test 
	bool safe = runReachability_obstacle(startState, timeToSafe, runtimeMs, startMs,v_x,v_y);
    DEBUG_PRINT("done, result = %s\n", safe ? "safe" : "unsafe");


    DEBUG_PRINT("Number of Iterations: %d\n",iterations_at_quit);
    // // print the hull 
	// println(&reach_hull);
	// deallocate_2darr(file_rows,file_columns);
	// deallocate_obstacles(obstacle_count);


	return 0;
}