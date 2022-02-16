
#include "util.h"
#include "face_lift.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/time.h>
// just for debug
static LiftingSettings errorPrintParams;
static bool errorParamsAssigned = false;


void set_error_print_params(LiftingSettings* set)
{
	errorParamsAssigned = true;
	errorPrintParams = *set;
}

void error_exit(const char* str)
{
	printf("Error: %s\n", str);

	// print the params that caused the error
	if (errorParamsAssigned)
	{
		printf("\nSettings:\n");
		printf("Reach Time = %f\n", errorPrintParams.reachTime);
		printf("Runtime = %i ms\n", errorPrintParams.maxRuntimeMilliseconds);
		printf("Init = ");
		println(&errorPrintParams.init);
	}
	else {
		printf("Error print params were not assigned.\n");
	}

	fflush(stdout);

	exit(1);
}


bool initialized = false;
long int startSec = 0;

long int milliseconds()
{
	startSec = 0;
	struct timeval now;
	gettimeofday(&now, NULL);

	if (!initialized)
	{
		initialized = true;
	}

	long int difSec = now.tv_sec - startSec;
	long int ms = now.tv_usec / 1000;
    long int ds = difSec * 1000 + ms;
	
	// printf("from milliseconds: %ld\n",ds);
	return ds;
}

long int milliseconds2(struct timeval * t1)
{
	struct timeval now;
	gettimeofday(&now, NULL);

	// printf("t1_sec: %lld, npw_sec: %lld\n\n",(long long) t1->tv_sec,(long long) now.tv_sec);

	long int elapsedTime;

	elapsedTime = (now.tv_sec - t1->tv_sec) * 1000.0;
	elapsedTime += (now.tv_usec - t1->tv_usec) / 1000.0;  

	return elapsedTime;
}
