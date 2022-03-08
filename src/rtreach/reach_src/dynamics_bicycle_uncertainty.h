// Patrick Musau
// 3-2022
// Bicycle Model Header file

// if this file is included in geometry.h, the controlled bicycle model dynamics will be compiled

#ifndef DYNAMICS_BICYCLE_MODEL_UNCERTAINTY_H_
#define DYNAMICS_BICYCLE_MODEL_UNCERTAINTY_H_


#include <stdbool.h>
#include "geometry.h"

double get_derivative_bounds_bicycle_uncertainty(HyperRectangle* rect, int faceIndex,REAL heading_input, REAL throttle, REAL parameter_uncertainty,REAL disturbance[][2]);

#endif
