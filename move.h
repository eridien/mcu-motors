#ifndef MOVE_H
#define	MOVE_H

#include <xc.h>
#include "types.h"
#include "motor.h"

// only 1/1 -> 1/8 ustep allowed 
// MS3 can be wired low in boards
#define MIN_USTEP 0
#define MAX_USTEP 3
extern const uint16 uStepPhaseMask[4];
extern const uint16 uStepDist[4];
extern const uint16 accelTable[8];

void checkMotor(void);
void moveCommand(bool noRules);

#endif	/* MOVE_H */

