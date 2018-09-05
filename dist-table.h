
#ifndef DISTTABLE_H
#define DISTTABLE_H

#include "types.h"

// see dist-table_calc-asm.js for code to generate disttable.asm

void initDistTable(void);

// table is 4 (ustep) by 8 (accel) by 64 (speed) -> 2048 entries
// return is 14-bit dist of decel
uint16 calcDist(uint8 ustep, uint8 accel, uint16 speed);

#endif /* DISTTABLE_H */
