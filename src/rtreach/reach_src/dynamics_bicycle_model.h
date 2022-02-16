// Patrick Musau
// 8-2020
// Bicycle Model Header file

// if this file is included in geometry.h, the controlled bicycle model dynamics will be compiled

#ifndef DYNAMICS_BICYCLE_MODEL_H_
#define DYNAMICS_BICYCLE_MODEL_H_


#include <stdbool.h>
#include "geometry.h"

double get_derivative_bounds_bicycle(HyperRectangle* rect, int faceIndex,REAL heading_input, REAL throttle);

#endif