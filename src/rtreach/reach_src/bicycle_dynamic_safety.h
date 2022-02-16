// Patrick Musau
// 09-2020
// Header file for safety checking of dynamic obstacles


#ifndef BICYCLE_DYNAMIC_SAFETY_H_
#define BICYCLE_DYNAMIC_SAFETY_H_

#include <stdio.h>
#include <stdbool.h>
#include <string.h> 
#include <stdlib.h>
#include <sys/time.h>
#include "geometry.h"

// array that will store the hyper-rectangle representations of the objects
// in this particular example they are conese but this can be easily converted to generic obstacles
extern double *** obstacles;
extern int obstacle_count;

bool check_safety(HyperRectangle* rect, double (*box)[2]); 
bool check_safety_obstacles(HyperRectangle* rect); 
void allocate_obstacles(int num_obstacles);
void append_obstacle(int index,double (*box)[2]);
void print_obstacles();
void deallocate_obstacles();
#endif 