
#ifndef CLOCK_H
#define	CLOCK_H

#include "motor.h"

// mcuClock is usually 30 usecs

#define setTicksSec() (PR1 = (16*mSet[0].val.mcuClock)-1)
extern volatile uint16 timeTicks; 
extern          uint16 clkTicksPerSec;
void clkInit(void);

#endif	/* CLOCK_H */

