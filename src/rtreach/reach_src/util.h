// Stanley Bak
// 4-2014
// Real-time Reachability utilities

#ifdef __cplusplus
extern "C" {
#endif

#ifndef UTIL_H_
#define UTIL_H_

#include "main.h"
#include "face_lift.h"
#include <sys/time.h> 

// print a string and exit
void error_exit(const char* str);

// sets the lifting params to print in case error_exit is called
void set_error_print_params(LiftingSettings* set);

// milliseconds timer
long int milliseconds(); // avr needs long (it has like 8-bit ints)
long int milliseconds2(struct timeval * t1); 

extern bool initialized;
extern long int startSec;

#endif

#ifdef __cplusplus
}
#endif

