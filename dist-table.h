
#ifndef DISTTABLE_H
#define DISTTABLE_H

#include "types.h"

// then insert the contents of dist-table-data.c into dist-table.c
extern const uint16 __attribute__((space(psv), address(0x8000))) disttable[2048];
  
// dist of decel in 1/8 steps
uint16 calcDist(uint16 accel, uint16 speed);


#endif /* DISTTABLE_H */
