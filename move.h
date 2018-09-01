#ifndef MOVE_H
#define	MOVE_H

#include <xc.h>
#include "types.h"
#include "motor.h"

// curUStep, only 1/2 -> 1/8 (1..3) is allowed 
#define MIN_USTEP 1
#define MAX_USTEP 3
const uint16 uStepPhaseMask[] = {0x0f, 0x07, 0x03, 0x01};
const uint16 uStepDist[]      = {   8,    4,    2,    1};


void calcMotion(void);
void moveCommand(void);
bool underAccellLimit(void);

#endif	/* MOVE_H */

