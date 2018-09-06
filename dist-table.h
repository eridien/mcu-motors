
#ifndef DISTTABLE_H
#define DISTTABLE_H

#include "types.h"

// see dist-table_calc-asm.js for code to generate disttable.asm

void initDistTable(void);

// dist of decel in 1/8 steps
uint16 calcDist(uint16 accel, uint16 speed);

#endif /* DISTTABLE_H */
