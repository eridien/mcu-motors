
#ifndef CLOCK_H
#define	CLOCK_H

#ifdef B1
#define CLK_PRESCALE          0  // 1:1 ( 2 usecs)
#define CLK_RATE             29  // wraps at count  30 (60 usecs, 16 khz)
#else
#define CLK_PRESCALE          0  // 1:1 ( 0.25 usecs)
#define CLK_RATE            159  // wraps at count 160 (40 usecs, 25 khz)
#endif

#define CLK_TICKS_PER_SEC 16666

extern volatile uint16 timeTicks; 

void clkInit(void);

#endif	/* CLOCK_H */

