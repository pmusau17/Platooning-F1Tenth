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
#include "obstacle_model_vis.h"
#include "simulate_obstacle.h"


const int state_n = 2; // state dimension
const int max_hyper_rectangles = 2000;

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

    // define array of hyperrectangles (array of structs)
    HyperRectangle hr_list[max_hyper_rectangles];
	HyperRectangle hr_list2[max_hyper_rectangles];

    // num_rects
    int rects = 0;
	int rects2 = 0;
	bool plot_all= true;


    getSimulatedSafeTime(startState,v_x,v_y);
    printf("\n");

    // run reachability analysis test 
	HyperRectangle reach_hull = runReachability_obstacle_vis(startState, timeToSafe, runtimeMs, startMs,v_x,v_y,hr_list,&rects,max_hyper_rectangles,plot_all);
	DEBUG_PRINT("Number of Iterations: %d\n",iterations_at_quit);
	HyperRectangle reach_hull2 = runReachability_obstacle_vis(startState, timeToSafe, runtimeMs, startMs,v_x+0.1,v_y+0.1,hr_list2,&rects2,max_hyper_rectangles,plot_all);
	DEBUG_PRINT("Number of Iterations: %d\n",iterations_at_quit);
    // DEBUG_PRINT("done, result = %s\n", safe ? "safe" : "unsafe");


    
    // // print the hull 
	printf("%d\n", rects);
	printf("%d\n", rects2);

	for(int i= 0; i<10; i++)
    {
		println(&hr_list[i]);
    }

	printf("\n\n States 2 \n");

	for(int i= 0; i<10; i++)
    {
		println(&hr_list2[i]);
    }

	printf("\n\n Final States\n");
	println(&reach_hull);
	println(&reach_hull2);
	return 0;
}