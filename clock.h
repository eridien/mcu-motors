
#ifndef CLOCK_H
#define	CLOCK_H

#define CLK_PRESCALE          0  // 1:1 ( 2 usecs)
#define CLK_RATE              9  // wraps at count 10 (20 usecs, 50 khz)
#define CLK_TICKS_PER_SEC 50000

extern volatile uint16 timeTicks; // units: 20 usecs, wraps on 1.31 secs

void clkInit(void);

#endif	/* CLOCK_H */

