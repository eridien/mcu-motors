
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "sens.h"

#ifdef B1
void sensInit(void) {
  // init DAC
  DAC1CON0bits.DAC1PSS1 = 0;   // max output is VDD  
  DAC1CON0bits.DAC1NSS  = 0;   // min output is GND  
  DAC1CON0bits.DAC1OE1  = 1;   // send output to dac1out pin (A0) shared with icp
  setDac(22);                  // 2.2V volt output (1ma through bridge)
  DAC1CON0bits.DAC1EN   = 1;   // enable dac
  
  // init ADC
  adcTRIS = 1;            // RB7 is input
  ANSELBbits.ANSB7 = 1;   // RB7 is analog
  ADCON0bits.CHS = 0x0f;  // select RB7 for ADC in
  ADCON1bits.ADPREF = 0;  // positive reference is VDD (negative is ground)
  ADCON1bits.ADFM = 1;    // right-justified output
  ADCON1bits.ADCS = 6;    // FOSC/64 clock, 2 us/bit (23 us total conversion)
  ADACT = 0;              // no automatic trigger, use startADC below
  ADIE = 0;               // no interrupts used
  ADCON0bits.ADON = 1;    // enable ADC 
}

// set dac output to val/10 volts
void setDac(uint16 val) {
  val = (val*35)/33; // measured
  DAC1CON1bits.DAC1R = (val > 31 ? 31 : val);
}

void startADC(void){
  ADCON0bits.GOnDONE = 1;
}

// ADRESH and ADRESL are read in interrupt -- see setSendBytesInt

#else
void sensInit(void) {}
void setDac(uint16 val) {}
void startADC(void) {}
uint16 getADC() {return 0;}
#endif
