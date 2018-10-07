
#ifndef CLOCK_H
#define	CLOCK_H

#include "motor.h"

#ifdef B1
#define setTicksSec() (TMR0H = (mSet[0].val.mcuClock/2)-1)     // wraps at ths count
#else
#define setTicksSec() (PR1 = (16*mSet[0].val.mcuClock)-1)
#endif
  
extern volatile uint16 timeTicks; 
extern          uint16 clkTicksPerSec;
void clkInit(void);

#endif	/* CLOCK_H */

