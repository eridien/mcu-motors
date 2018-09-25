
#ifndef DISTTABLE_H
#define DISTTABLE_H

#include "types.h"

#ifdef B1
// see dist-table_calc-asm.js for code to generate disttable.asm
void initDistTable(void);
#else
  extern const uint16 __attribute__((space(psv), address(0x8000))) disttable[2048];
#endif
  
// dist of decel in 1/8 steps
uint16 calcDist(uint16 accel, uint16 speed);


#endif /* DISTTABLE_H */
