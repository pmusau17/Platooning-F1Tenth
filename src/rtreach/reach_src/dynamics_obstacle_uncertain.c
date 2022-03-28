#include "dynamics_obstacle.h"
#include "dynamics_obstacle_model_uncertain.h"
#include "util.h"
#include "interval.h"

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#ifdef DYNAMICS_OBSTACLE_MODEL

// Dynamical model for the UUV Obstacles.
// In the Simulation Environment, they are essentially constant moving large boxes
// So the model I'm using is a simple point particle simulation with constant speed that is configurable via 
// input parameters.

// x' = v_x
// y' = v_y

// This file allows the velocity to lie within an interval

// implement the derivative using interval arithmetic
double get_derivative_obstacle_uncertain(HyperRectangle* rect, int faceIndex,REAL v_x[2], REAL v_y[2])
{
    int dim = faceIndex / 2;
	bool isMin = (faceIndex % 2) == 0;

    Interval rv;

    if( dim == 0 ) 
    {
        rv = new_interval(v_x[0],v_x[1]);
    }
    else if(dim == 1)
    {
        rv = new_interval(v_y[0],v_y[1]);
    }
    else 
    {
        printf("Error: Invalid Dimension");
        exit(0);
    }

    return isMin ? rv.min : rv.max;

}
#endif