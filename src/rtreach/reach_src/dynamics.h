// Stanley Bak
// 4-2014
// Dynamics header file for real-time reachability

#ifndef DYNAMICS_H_
#define DYNAMICS_H_

#include <stdbool.h>

#include "geometry.h"

/**
 * Get the bounds on the derivative in a region of space at a range of times
 */
REAL get_derivative_bounds(HyperRectangle* rect, int faceIndex);

#endif
