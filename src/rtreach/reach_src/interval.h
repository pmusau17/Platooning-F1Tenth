// Stanley Bak
// June 2015
// Interval arithmetic evaluation

#ifndef INTERVAL_H_
#define INTERVAL_H_

#include "geometry.h"

typedef struct Interval Interval;

Interval new_interval(double min, double max);
Interval new_interval_v(double val);
Interval add_interval(Interval i, Interval j); 
Interval sub_interval(Interval i, Interval j);
Interval mul_interval(Interval i, Interval j);
Interval div_interval(Interval i, Interval j);

Interval pow_interval(Interval i, int n); // a^n
Interval sin_interval(Interval i);
Interval cos_interval(Interval i);

#endif
