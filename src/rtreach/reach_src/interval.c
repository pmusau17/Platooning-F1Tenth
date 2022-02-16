#include "interval.h"
#include "stdio.h"
#include <math.h>

// for matlab, compiler may not have M_PI defined
#ifndef M_PI
#  define M_PI 3.141592653589793238
#endif

#define TWO_PI (2*M_PI)

Interval new_interval(double min, double max)
{
    Interval rv;
    rv.min = min;
    rv.max = max;

    return rv;
}

Interval new_interval_v(double val)
{
    Interval rv;
    rv.min = rv.max = val;

    return rv;
}

Interval add_interval(Interval i, Interval j)
{
    Interval rv;
    double a = i.min;
    double b = i.max;
    double c = j.min;
    double d = j.max;

    rv.min = a + c;
    rv.max = b + d;
    
    return rv;
}

Interval sub_interval(Interval i, Interval j)
{
    Interval rv;
    double a = i.min;
    double b = i.max;
    double c = j.min;
    double d = j.max;

    rv.min = a - d;
    rv.max = b - c;

    return rv;
}

Interval mul_interval(Interval i, Interval j)
{
    Interval rv;
    double a = i.min;
    double b = i.max;
    double c = j.min;
    double d = j.max;

    rv.min = fmin(fmin(a*c,a*d), fmin(b*c,b*d));
    rv.max = fmax(fmax(a*c,a*d), fmax(b*c,b*d));

    return rv;
}

Interval div_interval(Interval i, Interval j)
{
    double c = j.min;
    double d = j.max;

    return mul_interval(i, new_interval(1 / d, 1 / c));
}

Interval pow_interval(Interval i, int n) // a^n
{
    double a = i.min;
    double b = i.max;
    double c = 0;
    double d = 0;

    if (n % 2 == 1)
    {
        c = pow(a, n);
        d = pow(b, n);
    }
    else
    {
        if (a >= 0)
        {
            c = pow(a, n);
            d = pow(b, n);
        }
        else if (b < 0)
        {
            c = pow(b, n);
            d = pow(a, n);
        }
        else
        {
            c = 0;
            d = fmax(pow(a, n), pow(b, n));
        }
    }

    return new_interval(c, d);
}

Interval sin_interval(Interval i)
{
    double a = i.min;
    double b = i.max;
    double c = 0;
    double d = 0;

    if (floor((a-1.5*M_PI) / TWO_PI) != floor((b-1.5*M_PI) / TWO_PI))
        c = -1;
    else
        c = fmin(sin(a), sin(b));

    if (floor((a-0.5*M_PI) / TWO_PI) != floor((b-0.5*M_PI) / TWO_PI))
        d = 1;
    else
        d = fmax(sin(a), sin(b));

    return new_interval(c, d);
}

Interval cos_interval(Interval i)
{
    double a = i.min;
    double b = i.max;
    double c = 0;
    double d = 0;

    if (floor((a+M_PI) / TWO_PI) != floor((b+M_PI) / TWO_PI))
        c = -1;
    else
        c = fmin(cos(a), cos(b));

    if (floor(a / TWO_PI) != floor(b / TWO_PI))
        d = 1;
    else
        d = fmax(cos(a), cos(b));

    return new_interval(c, d);
}
