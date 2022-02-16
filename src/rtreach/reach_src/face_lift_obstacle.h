// Patrick Musau
// 12-2020
// Real-time face lifting main algorithm header

#ifndef FACE_LIFT_H_
#define FACE_LIFT_H_

#include <stdbool.h>

#include "main.h"
#include "geometry.h"

extern int iterations_at_quit;
typedef struct LiftingSettings
{
	HyperRectangle init;

	REAL reachTime; // total reach time

	REAL initialStepSize; // the initial size of the steps to use
	REAL maxRectWidthBeforeError; // maximum allowed rectangle size

	int maxRuntimeMilliseconds; // maximum runtime in milliseconds

	// called at the intermediate times
	// return true if this rectangle is satisfactory (for safety or whatever)
	bool (*reachedAtIntermediateTime)(HyperRectangle* r, HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate);

	// called at the final time
	// return true if the system is satisfactory (for liveness or whatever)
	bool (*reachedAtFinalTime)(HyperRectangle* r, HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate);

	// called whenever we restart the computation after refining
	void (*restartedComputation)(int  *total_intermediate);

} LiftingSettings;

#endif
