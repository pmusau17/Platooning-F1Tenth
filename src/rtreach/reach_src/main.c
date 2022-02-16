// example call: ./bicycle 100 0.0 0.0 0.0 0.0 16.0 0.26666
// example call output:
// started!
// Argc: 6
// runtime: 100 ms
// x_0[0]: -0.100000
// x_0[1]: 0.000000
// x_0[2]: 0.000000
// x_0[3]: 1.100000
// u_0[0]: 16.0
// u_1[1]: 0.266

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "util.h"
#include "main.h"
#include "bicycle_model.h"
#include "bicycle_safety.h"

const int state_n = 4; // state dimension

// This particular example also needs to know where the walls are
const char * filepath= "../ros_src/rtreach/obstacles/porto_obstacles.txt";

int main( int argc, const char* argv[] )
{
	DEBUG_PRINT("started!\n\r");

	int runtimeMs = 0;
	REAL startState[4] = {0.0, 0.0, 0.0, 0.0};
    REAL control_input[2] = {0.0,0.0};

	DEBUG_PRINT("Argc: %d\n\r", argc);

	if (argc < 6) {
		printf("Error: not enough input arguments!\n\r");
		return 0;
	}
	else {
		runtimeMs = atoi(argv[1]);
		startState[0] = atof(argv[2]);
		startState[1] = atof(argv[3]);
		startState[2] = atof(argv[4]);
		startState[3] = atof(argv[5]);
        control_input[0] = atof(argv[6]);
        control_input[1] = atof(argv[7]);
		DEBUG_PRINT("runtime: %d ms\n\rx_0[0]: %f\n\rx_0[1]: %f\n\rx_0[2]: %f\n\rx_0[3]: %f\n\ru_0[0]: %f\n\ru_0[1]: %f\n\r\n", runtimeMs, startState[0], startState[1], startState[2], startState[3],control_input[0],control_input[1]);
	}

    REAL delta = control_input[1];
    REAL u = control_input[0];
    // simulate the car with a constant input passed from the command line
    getSimulatedSafeTime(startState,delta,u);
    printf("\n");

	// location of obstacles in our scenario
	int num_obstacles = 5;
	double points[5][2] = {{2.0,2.0},{4.7,2.7},{11.36,-1.46},{3.0,6.4},{-9.64,2.96}};
	allocate_obstacles(num_obstacles,points);
    // simTime 
    REAL timeToSafe = 2.0;

    // startMs 
    int startMs = 0;

	// load the wall points into the global variable
	load_wallpoints(filepath,true);
    
    // run reachability analysis test 
	bool safe = runReachability_bicycle(startState, timeToSafe, runtimeMs, startMs,delta,u);
	//int runtimeMs = 20; // run for 20 milliseconds
	DEBUG_PRINT("Number of Iterations: %d\n",iterations_at_quit);
	DEBUG_PRINT("done, result = %s\n", safe ? "safe" : "unsafe");
	deallocate_2darr(file_rows,file_columns);
	deallocate_obstacles(obstacle_count);

	return 0;
}