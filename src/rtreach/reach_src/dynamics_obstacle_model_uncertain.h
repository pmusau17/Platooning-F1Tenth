// Patrick Musau
// 03-2022
// Obstacle Model Header file


#ifndef DYNAMICS_OBSTACLE_MODEL_UNCERTAIN_H_
#define DYNAMICS_OBSTACLE_MODEL_UNCERTAIN_H_


#include <stdbool.h>
#include "geometry.h"

// This is the most basic model I could have used 

double get_derivative_obstacle_uncertain(HyperRectangle* rect, int faceIndex,REAL v_x[2], REAL v_y[2]);

#endif