
#ifndef CLOCK_H
#define	CLOCK_H

#define CLK_PRESCALE          0  // 1:1 ( 2 usecs)
#define CLK_RATE             19  // wraps at count 20 (40 usecs, 25 khz)
#define CLK_TICKS_PER_SEC 25000

extern volatile uint16 timeTicks; 

void clkInit(void);

#endif	/* CLOCK_H */

