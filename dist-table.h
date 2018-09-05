
#ifndef DISTTABLE_H
#define DISTTABLE_H

#include "types.h"

void initDistTable();

// table is 4 (ustep) by 8 (accel) by 64 (speed) -> 2048 entries
// return is 14-bit dist of decel
void calcDist(uint8 ustep, uint8 accel, uint16 speed);

#endif /* DISTTABLE_H */
