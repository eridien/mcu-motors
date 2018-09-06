#ifndef MOVE_H
#define	MOVE_H

#include <xc.h>
#include "types.h"
#include "motor.h"

// only 1/1 -> 1/8 ustep allowed 
// so MS3 can be wired low in boards
#define MIN_USTEP 0
#define MAX_USTEP 3
const uint16 uStepPhaseMask[] = {0x07, 0x03, 0x01, 0x00};
const uint16 uStepDist[]      = {   8,    4,    2,    1};
const uint16 accelTable[8] = 
       {4000, 8000, 16000, 24000, 32000, 40000, 50000, 60000};

void checkMotor(void);
void moveCommand(void);

#endif	/* MOVE_H */

