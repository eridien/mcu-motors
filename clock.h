
#ifndef CLOCK_H
#define	CLOCK_H

#include "motor.h"


#ifdef B1
#define CLK_PRESCALE          0  // 1:1 ( 2 usecs)
#else
#define CLK_PRESCALE          0  // 1:1 ( 0.25 usecs)
#endif

#define CLK_TICKS_PER_SEC ((uint16) (1000000 / mSet[0].val.mcuClock))

#ifdef B1
#define setTicksSec() (PR1 = (mSet[0].val.mcuClock/2)-1)     // wraps at ths count
#else
#define setTicksSec() (PR1 = (16*mSet[0].val.mcuClock)-1)
#endif
  
extern volatile uint16 timeTicks; 

void clkInit(void);

#endif	/* CLOCK_H */

