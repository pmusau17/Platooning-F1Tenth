// example call: ./bicycle_param 100 0.0 0.0 0.0 0.0 16.0 0.26666
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
#include "bicycle_model_parametrizeable.h"

const int max_hyper_rectangles = 2000;

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


    HyperRectangle hr_list[max_hyper_rectangles];
    int rects = 0;
    bool plot_all= true;


    // simulate the car with a constant input passed from the command line
    getSimulatedSafeTime(startState,delta,u);
    printf("\n");

    // simTime 
    REAL timeToSafe = 2.0;

    // startMs 
    int startMs = 0;

    // run reachability analysis test 
	bool safe = runReachability_bicycle_dyn(startState, timeToSafe, runtimeMs, startMs,delta,u,hr_list,&rects,max_hyper_rectangles,plot_all);
    DEBUG_PRINT("Number of Iterations: %d\n",iterations_at_quit);
    DEBUG_PRINT("done, result = %s\n", safe ? "safe" : "unsafe");

    // // print the hull 
	printf("%d\n", rects);
	for(int i= 0; i<10; i++)
    {
		println(&hr_list[i]);
    }
    if(rects<max_hyper_rectangles){
        println(&hr_list[rects-2]);
    }
    else
        println(&hr_list[max_hyper_rectangles-2]);

    


	return 0;
}
    