
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
  uint16 clkUsec =  settingsInit[mcuClockSettingIdx];
#ifdef B1
  // timer 0
  T0ASYNC             =  0;              // sync clock
  T016BIT             =  0;              // 8-bit counter
  T0CON1bits.T0CS     =  3;              // src clk is HFINTOSC (32 Mhz)
  T0CON1bits.T0CKPS   =  6;              // prescaler is 1:64 (2 usecs)
  TMR0H               =  (clkUsec/2)-1;  // wraps at this count
  T0CON0bits.T0OUTPS  =  0;              // ouput post-scaler is 1:1 (clkUsec)
  TMR0IF              =  0;              // int flag
  T0EN                =  1;              // enable timer0
  TMR0IE              =  1;              // enable timer int
#else
  // timer 0
  _TSYNC              =  0;              // sync clock
  _TCS                =  0;              // Clock Source Internal clock (FOSC/2)
  _TCKPS              =  0;              // prescaler  is 1:1 (0.25 usecs)
  PR1                 =  (4*clkUsec)-1; // wraps at ths count
  _T1IF               =  0;              // int flag
  _TON                =  1;              // enable timer0
  _T1IE               =  1;              // enable timer int
#endif
}

volatile uint16 timeTicks;     // units: 20 usecs, wraps on 1.31 secs

// clock interrupt routine is in motor.c
