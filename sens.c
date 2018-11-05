
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "sens.h"

#ifdef B1
void sensInit(void) {
  DAC1CON0bits.DAC1PSS1 = 0;   // max output is Vdd  
  DAC1CON0bits.DAC1NSS  = 0;   // min output is GND  
  DAC1CON0bits.DAC1OE1  = 1;   // send output to dac1out pin (A0) shared ith icp
  setDac(10);                  // start with 1 volt output
  DAC1CON0bits.DAC1EN   = 1;   // enable dac
}

// set dac output to val/10 volts
void setDac(uint16 val) {
  val = (val*34)/33; // measured
  DAC1CON1bits.DAC1R = (val > 31 ? 31 : val);
}
#else
void sensInit(void) {}
void setDac(uint16 val) {}
#endif
