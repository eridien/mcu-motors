
#include <xc.h>
#include "types.h"
#include "clock.h"
#include "pins.h"
#include "motor.h"

void clkInit(void) {
  uint16 clkUsec =  settingsInit[mcuClockSettingIdx];
#ifdef B1
  T0ASYNC             =  0;            // sync clock
  T016BIT             =  0;            // 8-bit counter
  T0CON1bits.T0CS     =  5;            // src clk is MFINTOSC (500 khz)
  T0CON1bits.T0CKPS   =  CLK_PRESCALE; // prescaler  is 1:1 ( 2 usecs)
  PR1                 =  (clkUsec/2)-1;// wraps at ths count
  TMR0IF              =  0;            // int flag
  T0EN                =  1;            // enable timer0
  TMR0IE              =  1;            // enable timer int
#else
  _TSYNC               =  0;            // sync clock
  _TCS                 =  0;            // Clock Source Internal clock (FOSC/2)
  _TCKPS               =  CLK_PRESCALE; // prescaler  is 1:1 ( .25 usecs)
  PR1                  =  (16*clkUsec)-1;// wraps at ths count
  _T1IF                =  0;            // int flag
  _TON                 =  1;            // enable timer0
  _T1IE                =  1;            // enable timer int
#endif
}

volatile uint16 timeTicks;     // units: 20 usecs, wraps on 1.31 secs

// clock interrupt routine is in motor.c
