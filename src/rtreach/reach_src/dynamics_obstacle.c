#include "dynamics_obstacle.h"
#include "dynamics_obstacle_model.h"
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


// implement the derivative using interval arithmetic
double get_derivative_obstacle(HyperRectangle* rect, int faceIndex,REAL v_x, REAL v_y)
{
    int dim = faceIndex / 2;
	bool isMin = (faceIndex % 2) == 0;

    Interval rv;

    if( dim == 0 ) 
    {
        rv = new_interval_v(v_x);
    }
    else if(dim == 1)
    {
        rv = new_interval_v(v_y);
    }
    else 
    {
        printf("Error: Invalid Dimension");
        exit(0);
    }

    return isMin ? rv.min : rv.max;

}
#endif