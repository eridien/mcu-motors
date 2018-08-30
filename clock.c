
#include <xc.h>
#include "types.h"
#include "clock.h"
#include "pins.h"
#include "motor.h"

void clkInit(void) {
  T0ASYNC             =  0;   // sync clock
  T016BIT             =  0;   // 8-bit counter
  T0CON1bits.T0CS     =  5;   // src clk is MFINTOSC (500 khz)
  T0CON1bits.T0CKPS   =  0;   // prescaler  is 1:1 ( 2 usecs)
  TMR0H               =  9;   // wraps at count 10 (20 usecs, 50 khz)
  TMR0IF              =  0;   // int flag
  T0EN                =  1;   // enable timer0
  TMR0IE              =  1;   // enable timer int
}

volatile uint16 timeTicks;     // units: 20 usecs, wraps on 1.31 secs

