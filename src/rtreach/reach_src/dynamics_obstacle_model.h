// Patrick Musau
// 10-2020
// Obstacle Model Header file


#ifndef DYNAMICS_OBSTACLE_MODEL_H_
#define DYNAMICS_OBSTACLE_MODEL_H_


#include <stdbool.h>
#include "geometry.h"

// This is the most basic model I could have used 

double get_derivative_obstacle(HyperRectangle* rect, int faceIndex,REAL v_x, REAL v_y);

#endif