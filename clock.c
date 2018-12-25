
#include <xc.h>
#include "types.h"
#include "clock.h"
#include "pins.h"
#include "motor.h"
/*
 * 3 => 14.6 usecs
 * 4 => 14.6
 * 6 => 40
 * 8 => 160 usecs
 * 
 */

uint16 clkTicksPerSec;

void clkInit(void) {
  uint16 clkUsec =  DEF_MCU_CLK;
  // timer 0
  _TSYNC              =  0;              // sync clock
  _TCS                =  0;              // Clock Source Internal clock (FOSC/2)
  _TCKPS              =  0;              // prescaler  is 1:1 (0.25 usecs)
  PR1                 =  (4*clkUsec)-1; // wraps at ths count
  _T1IF               =  0;              // int flag
  _TON                =  1;              // enable timer0
  _T1IE               =  1;              // enable timer int
}

volatile uint16 timeTicks;     // units: 20 usecs, wraps on 1.31 secs

// clock interrupt routine is in motor.c
