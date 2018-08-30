
#ifndef CLOCK_H
#define	CLOCK_H

extern volatile uint16 timeTicks; // units: 20 usecs, wraps on 1.31 secs

void clkInit(void);

#endif	/* CLOCK_H */

